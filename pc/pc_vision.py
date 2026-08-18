"""
=========================================================================================
PC VISION CONTROLLER MODULE (pc_vision.py)
-----------------------------------------------------------------------------------------
Single deterministic vision pipeline for the Webots ROVE MVP.

Public API intentionally unchanged:
    VisionController
    VisionController.map_error_to_angle()
    VisionController.detect_obstacle()
    VisionController.process_frame()

There is exactly ONE obstacle detector in this module.
=========================================================================================
"""

import cv2  # type: ignore
import time
import numpy as np
from ultralytics import YOLOE

class VisionController:  # type: ignore

    def __init__(self, scale_factor=0.5):  # type: ignore

        # =====================================================================
        # BASIC VEHICLE / STEERING PARAMETERS
        # =====================================================================

        self.scale_factor = scale_factor

        self.servo_center = 90
        self.last_valid_angle = 90

        # =====================================================================
        # LINE-LOSS SAFETY STATE
        # =====================================================================

        self.state = "DRIVE"

        self.lost_line_timestamp = 0.0
        self.reverse_start_timestamp = 0.0

        # =====================================================================
        # OBSTACLE STATE
        # =====================================================================

        self.obstacle_detected = False
        self.obstacle_cx = 0
        self.obstacle_area = 0
        self.obstacle_direction = "NONE"

        # =====================================================================
        # OBSTACLE AVOIDANCE STATE
        # =====================================================================

        self.avoidance_active = False
        self.avoidance_direction = "NONE"
        self.avoidance_start_time = 0.0

        # Controlled lateral manoeuvre.
        self.avoidance_duration = 0.9
        self.avoidance_recenter_duration = 0.35

        # =====================================================================
        # POST-AVOIDANCE LINE REACQUISITION
        # =====================================================================

        self.reacquire_active = False
        self.reacquire_start_time = 0.0
        self.reacquire_timeout = 2.5

        # Preserve the original avoidance direction so the recovery search
        # knows which way the vehicle moved around the obstacle.
        self.recovery_direction = "NONE"

        # =====================================================================
        # STEERING FILTER
        # =====================================================================

        self.filtered_angle = float(self.servo_center)

        # Lower value = smoother steering.
        # This prevents rapid left/right oscillation.
        self.steering_alpha = 0.20

        # =====================================================================
        # OBSTACLE TEMPORAL CONFIRMATION
        # =====================================================================

        # Do not react to one noisy frame.
        self.obstacle_confirm_count = 0
        self.obstacle_confirm_required = 2

        # =====================================================================
        # LINE TRACKING MEMORY
        # =====================================================================

        self.last_line_cx = None
        self.last_line_error = 0.0
        self.line_missed_frames = 0

        # =====================================================================
        # PRIMARY AI OBJECT DETECTION
        # =====================================================================

        self.object_detector = YOLOE(
            "yoloe-11s-seg.pt"
        )

        self.object_detector.set_classes([
            "person",
            "pedestrian",
            "car",
            "bus",
            "truck",
            "motorcycle",
            "bicycle",
            "tree",
            "fallen tree",
            "road debris",
            "rock",
            "barrier",
            "traffic cone",
            "animal",
            "obstacle"
        ])

        # Minimum confidence for an AI object detection.
        self.object_detection_conf = 0.15

        # Vehicle path width as a fraction of camera width.
        self.object_path_width_ratio = 0.30

        # Object information exposed to the rest of the system.
        self.detected_object_type = "NONE"
        self.detected_object_confidence = 0.0
        self.detected_object_bbox = None

    # =========================================================================
    # STEERING MAPPING
    # =========================================================================

    def map_error_to_angle(self, error):  # type: ignore
        """
        Convert horizontal line-position error into steering angle.

        Positive error:
            line is to the RIGHT
            -> steer RIGHT

        Negative error:
            line is to the LEFT
            -> steer LEFT
        """

        # Moderate proportional gain.
        #
        # The previous aggressive steering behaviour came partly from
        # reacting too strongly to noisy pixel positions.
        kp = 0.30

        raw_angle = (
            self.servo_center +
            (float(error) * kp)
        )

        # Keep requested steering inside controlled operating range.
        raw_angle = max(
            45.0,
            min(135.0, raw_angle)
        )

        # Temporal low-pass filtering.
        self.filtered_angle += (
            self.steering_alpha *
            (raw_angle - self.filtered_angle)
        )

        angle = int(
            round(self.filtered_angle)
        )

        # Final mechanical protection.
        return max(
            45,
            min(135, angle)
        )
    # =========================================================================
    # PRIMARY AI OBJECT DETECTOR
    # =========================================================================
    #newly added
    def detect_objects(self, frame, line_cx=None):  # type: ignore
        """
        Primary object-first detection pipeline.

        The AI detector identifies objects first.

        The object becomes an obstacle ONLY when its bounding box
        overlaps the vehicle's forward path.

        Returns:
            obstacle_detected
            obstacle_cx
            obstacle_area
            obstacle_direction
        """

        if frame is None or frame.size == 0:

            self.detected_object_type = "NONE"
            self.detected_object_confidence = 0.0
            self.detected_object_bbox = None

            return (
                False,
                0,
                0,
                "NONE"
            )


        # =====================================================================
        # RUN AI OBJECT DETECTION
        # =====================================================================

        results = list(self.object_detector.predict(
            source=frame,
            imgsz=640,
            conf=self.object_detection_conf,
            verbose=False
        ))


        if not results:

            self.detected_object_type = "NONE"
            self.detected_object_confidence = 0.0
            self.detected_object_bbox = None

            return (
                False,
                0,
                0,
                "NONE"
            )


        result = results[0]


        if (
            result.boxes is None
            or
            len(result.boxes) == 0
        ):

            self.detected_object_type = "NONE"
            self.detected_object_confidence = 0.0
            self.detected_object_bbox = None

            return (
                False,
                0,
                0,
                "NONE"
            )


        # =====================================================================
        # EXTRACT AI DETECTIONS
        # =====================================================================

        boxes = (
            result.boxes.xyxy
            .cpu()
            .numpy()
        )

        confidences = (
            result.boxes.conf
            .cpu()
            .numpy()
        )

        classes = (
            result.boxes.cls
            .cpu()
            .numpy()
        )


        height, width = (
            frame.shape[:2]
        )


        # =====================================================================
        # VEHICLE PATH CENTRE
        # =====================================================================

        if line_cx is not None:

            path_center = int(
                line_cx
            )

        else:

            path_center = (
                width // 2
            )


        # =====================================================================
        # VEHICLE PATH WIDTH
        # =====================================================================

        path_half_width = max(
            12,
            int(
                width *
                self.object_path_width_ratio
            )
        )


        path_left = max(
            0,
            path_center -
            path_half_width
        )


        path_right = min(
            width - 1,
            path_center +
            path_half_width
        )


        # =====================================================================
        # SELECT BEST PATH-BLOCKING OBJECT
        # =====================================================================

        best_object = None
        best_score = -1.0


        for (
            box,
            confidence,
            class_id
        ) in zip(
            boxes,
            confidences,
            classes
        ):

            x1, y1, x2, y2 = [
                int(value)
                for value in box
            ]


            raw_name = result.names.get(
                int(class_id),
                "unknown"
            )


            raw_name = str(
                raw_name
            ).lower()


            # -------------------------------------------------------------
            # DISPLAY ALL AI-DETECTED OBJECTS
            # -------------------------------------------------------------

            cv2.rectangle(
                frame,
                (x1, y1),
                (x2, y2),
                (255, 180, 0),
                1
            )


            cv2.putText(
                frame,
                (
                    f"{raw_name} "
                    f"{float(confidence):.2f}"
                ),
                (
                    x1,
                    max(
                        12,
                        y1 - 4
                    )
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.32,
                (255, 180, 0),
                1
            )


            # -------------------------------------------------------------
            # IGNORE VERY DISTANT / UPPER-FRAME OBJECTS
            # -------------------------------------------------------------

            if (
                y2 <
                int(height * 0.30)
            ):
                continue


            # -------------------------------------------------------------
            # CHECK WHETHER OBJECT OVERLAPS VEHICLE PATH
            # -------------------------------------------------------------

            overlaps_path = (
                x2 >= path_left
                and
                x1 <= path_right
            )


            # Object exists but is not on our path.
            if not overlaps_path:

                continue


            # -------------------------------------------------------------
            # APPROXIMATE OBJECT PROXIMITY
            # -------------------------------------------------------------

            object_height = max(
                1,
                y2 - y1
            )


            proximity_score = (
                (
                    y2 /
                    float(height)
                )
                * 0.60
                +
                min(
                    object_height /
                    float(height),
                    1.0
                )
                * 0.40
            )


            score = (
                float(confidence)
                *
                proximity_score
            )


            if score > best_score:

                best_score = score

                best_object = (
                    x1,
                    y1,
                    x2,
                    y2,
                    raw_name,
                    float(confidence)
                )


        # =====================================================================
        # DRAW VEHICLE PATH
        # =====================================================================

        cv2.rectangle(
            frame,
            (
                path_left,
                int(
                    height * 0.45
                )
            ),
            (
                path_right,
                height - 1
            ),
            (255, 255, 0),
            1
        )


        # =====================================================================
        # NO PATH-BLOCKING OBJECT
        # =====================================================================

        if best_object is None:

            self.detected_object_type = "NONE"
            self.detected_object_confidence = 0.0
            self.detected_object_bbox = None

            return (
                False,
                0,
                0,
                "NONE"
            )


        # =====================================================================
        # PATH-BLOCKING OBJECT FOUND
        # =====================================================================

        (
            x1,
            y1,
            x2,
            y2,
            raw_name,
            confidence
        ) = best_object


        obstacle_cx = (
            x1 +
            (
                (x2 - x1)
                // 2
            )
        )


        obstacle_area = (
            (x2 - x1)
            *
            (y2 - y1)
        )


        direction = (
            "LEFT"
            if obstacle_cx < path_center
            else
            "RIGHT"
        )


        self.detected_object_type = (
            raw_name
        )


        self.detected_object_confidence = (
            confidence
        )


        self.detected_object_bbox = (
            x1,
            y1,
            x2,
            y2
        )


        # =====================================================================
        # HIGHLIGHT ONLY THE PATH-BLOCKING OBJECT
        # =====================================================================

        cv2.rectangle(
            frame,
            (x1, y1),
            (x2, y2),
            (0, 0, 255),
            2
        )


        cv2.putText(
            frame,
            (
                f"PATH BLOCKED: "
                f"{raw_name} "
                f"{confidence:.2f}"
            ),
            (5, 118),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.40,
            (0, 0, 255),
            1
        )


        return (
            True,
            obstacle_cx,
            int(obstacle_area),
            direction
        )#newly added

    # =========================================================================
    # SINGLE OBSTACLE DETECTOR
    # =========================================================================

    def detect_obstacle(self, frame, yellow_mask):  # type: ignore
        """
        Detect ONE relevant red/brown obstacle in the forward road corridor.

        This function ONLY detects.

        It does NOT:
            - steer
            - drive
            - stop
            - perform avoidance

        Avoidance is handled centrally inside process_frame().
        """

        # Signature retained for compatibility.
        del yellow_mask

        if frame is None or frame.size == 0:
            self.obstacle_detected = False
            self.obstacle_cx = 0
            self.obstacle_area = 0
            self.obstacle_direction = "NONE"

            return False, 0, 0, "NONE"

        # -------------------------------------------------------------
        # Convert camera image to HSV.
        # -------------------------------------------------------------

        hsv = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2HSV
        )

        height, width = hsv.shape[:2]

        # -------------------------------------------------------------
        # IMPORTANT:
        # Only detect RED / ORANGE / BROWN.
        #
        # Yellow road line:
        #     H ~= 18-40
        #
        # Obstacle:
        #     H ~= 0-18
        #
        # This prevents the yellow line from becoming an obstacle.
        # -------------------------------------------------------------

        lower_obstacle = np.array(
            [0, 100, 40],
            dtype=np.uint8
        )

        upper_obstacle = np.array(
            [17, 255, 220],
            dtype=np.uint8
        )

        obstacle_mask = cv2.inRange(
            hsv,
            lower_obstacle,
            upper_obstacle
        )

        # -------------------------------------------------------------
        # Only inspect lower/forward road region.
        # -------------------------------------------------------------

        roi_start = int(
            height * 0.38
        )

        road_mask = np.zeros_like(
            obstacle_mask
        )

        road_mask[
            roi_start:height,
            :
        ] = obstacle_mask[
            roi_start:height,
            :
        ]

        # -------------------------------------------------------------
        # Remove small noise.
        # -------------------------------------------------------------

        kernel = np.ones(
            (3, 3),
            np.uint8
        )

        road_mask = cv2.morphologyEx(
            road_mask,
            cv2.MORPH_OPEN,
            kernel
        )

        road_mask = cv2.morphologyEx(
            road_mask,
            cv2.MORPH_CLOSE,
            kernel,
            iterations=2
        )

        # -------------------------------------------------------------
        # Find contours.
        # -------------------------------------------------------------

        contours, _ = cv2.findContours(
            road_mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        best = None
        best_area = 0.0

        # -------------------------------------------------------------
        # Select ONE best obstacle.
        # -------------------------------------------------------------

        for contour in contours:

            area = float(
                cv2.contourArea(contour)
            )

            # Reject tiny noise.
            if area < 35.0:
                continue

            x, y, w, h = cv2.boundingRect(
                contour
            )

            # Reject tiny objects.
            if w < 6 or h < 6:
                continue

            # Reject very wide/shallow scenery.
            if (
                w > int(width * 0.75)
                and
                h < int(height * 0.18)
            ):
                continue

            if area > best_area:

                best_area = area

                best = (
                    x,
                    y,
                    w,
                    h
                )

        # -------------------------------------------------------------
        # Nothing valid detected.
        # -------------------------------------------------------------

        if best is None:

            self.obstacle_detected = False
            self.obstacle_cx = 0
            self.obstacle_area = 0
            self.obstacle_direction = "NONE"

            return False, 0, 0, "NONE"

        # -------------------------------------------------------------
        # Calculate selected obstacle geometry.
        # -------------------------------------------------------------

        x, y, w, h = best

        obstacle_cx = (
            x +
            (w // 2)
        )

        camera_center = (
            width // 2
        )

        # -------------------------------------------------------------
        # Vehicle forward corridor.
        # -------------------------------------------------------------

        path_half_width = max(
            8,
            int(width * 0.25)
        )

        path_left = (
            camera_center -
            path_half_width
        )

        path_right = (
            camera_center +
            path_half_width
        )

        obstacle_left = x
        obstacle_right = x + w

        overlaps_path = (
            obstacle_right >= path_left
            and
            obstacle_left <= path_right
        )

        # -------------------------------------------------------------
        # Object exists but is outside our path.
        # -------------------------------------------------------------

        if not overlaps_path:

            self.obstacle_detected = False
            self.obstacle_cx = obstacle_cx
            self.obstacle_area = int(best_area)
            self.obstacle_direction = "SIDE"

            return (
                False,
                obstacle_cx,
                int(best_area),
                "SIDE"
            )

        # -------------------------------------------------------------
        # Object is actually blocking the forward corridor.
        # -------------------------------------------------------------

        direction = (
            "LEFT"
            if obstacle_cx < camera_center
            else
            "RIGHT"
        )

        self.obstacle_detected = True
        self.obstacle_cx = obstacle_cx
        self.obstacle_area = int(best_area)
        self.obstacle_direction = direction

        # -------------------------------------------------------------
        # Draw ONLY the selected obstacle.
        # -------------------------------------------------------------

        cv2.rectangle(
            frame,
            (x, y),
            (x + w, y + h),
            (0, 0, 255),
            2
        )

        cv2.putText(
            frame,
            "OBSTACLE",
            (
                x,
                max(15, y - 4)
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 0, 255),
            1
        )

        return (
            True,
            obstacle_cx,
            int(best_area),
            direction
        )

    # =========================================================================
    # MAIN FRAME PROCESSOR
    # =========================================================================

    def process_frame(self, frame):
        """
        Process one camera frame.

        Returns:
            target_angle
            engine_state
            processed_frame
        """

        # =====================================================================
        # FRAME VALIDATION
        # =====================================================================

        if frame is None or frame.size == 0:

            return (
                self.servo_center,
                "STOP",
                frame
            )


        # =====================================================================
        # RESIZE ONCE
        # =====================================================================

        small_frame = cv2.resize(
            frame,
            (0, 0),
            fx=self.scale_factor,
            fy=self.scale_factor,
            interpolation=cv2.INTER_AREA
        )

        height, width = (
            small_frame.shape[:2]
        )


        if (
            height < 12
            or
            width < 12
        ):

            return (
                self.servo_center,
                "STOP",
                small_frame
            )


        now = time.time()


        # =====================================================================
        # CAMERA SANITY CHECK
        # =====================================================================

        gray = cv2.cvtColor(
            small_frame,
            cv2.COLOR_BGR2GRAY
        )

        brightness = float(
            cv2.mean(gray)[0]
        )


        if (
            brightness < 18
            or
            brightness > 247
        ):

            self.state = "CAMERA_FAULT"

            cv2.putText(
                small_frame,
                "CAMERA FAULT - STOP",
                (5, 15),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.38,
                (0, 0, 255),
                1
            )

            return (
                self.servo_center,
                "STOP",
                small_frame
            )


        # =====================================================================
        # ONE HSV CONVERSION
        # =====================================================================

        hsv = cv2.cvtColor(
            small_frame,
            cv2.COLOR_BGR2HSV
        )


        # =====================================================================
        # YELLOW LINE DETECTION
        # =====================================================================

        lower_yellow = np.array(
            [18, 90, 75],
            dtype=np.uint8
        )

        upper_yellow = np.array(
            [40, 255, 255],
            dtype=np.uint8
        )

        yellow_mask = cv2.inRange(
            hsv,
            lower_yellow,
            upper_yellow
        )


        # =====================================================================
        # LINE ROI
        # =====================================================================
        #
        # Only lower road area is used for line tracking.
        # =====================================================================

        roi_start = int(
            height * 0.30
        )

        roi_mask = np.zeros_like(
            yellow_mask
        )

        roi_mask[
            roi_start:,
            :
        ] = yellow_mask[
            roi_start:,
            :
        ]


        # =====================================================================
        # CLEAN YELLOW MASK
        # =====================================================================

        kernel = np.ones(
            (3, 3),
            np.uint8
        )

        roi_mask = cv2.morphologyEx(
            roi_mask,
            cv2.MORPH_OPEN,
            kernel
        )

        roi_mask = cv2.morphologyEx(
            roi_mask,
            cv2.MORPH_CLOSE,
            kernel,
            iterations=2
        )


        # =====================================================================
        # FIND YELLOW LINE COMPONENTS
        # =====================================================================

        contours, _ = cv2.findContours(
            roi_mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )


        line_contour = None
        line_score = -1.0


        # Minimum contour area.
        min_line_area = max(
            4.0,
            width * height * 0.0012
        )


        # =====================================================================
        # SELECT ONE REAL LINE COMPONENT
        # =====================================================================

        for contour in contours:

            area = float(
                cv2.contourArea(contour)
            )

            if area < min_line_area:
                continue


            x, y, w, h = cv2.boundingRect(
                contour
            )


            if (
                w < 2
                or
                h < 3
            ):
                continue


            # -----------------------------------------------------------------
            # ZEBRA / PEDESTRIAN CROSSING PROTECTION
            # -----------------------------------------------------------------
            #
            # Zebra stripes are generally wide and horizontally oriented.
            #
            # Do NOT use a wide shallow yellow region as the guide line.
            # -----------------------------------------------------------------

            aspect_ratio = (
                w /
                max(
                    float(h),
                    1.0
                )
            )

            if (
                aspect_ratio > 4.0
                and
                h < height * 0.20
            ):
                continue


            # -----------------------------------------------------------------
            # Component position scoring
            # -----------------------------------------------------------------

            bottom = (
                y +
                h
            )

            bottom_score = (
                bottom /
                float(height)
            )

            vertical_score = min(
                h /
                max(
                    float(height) * 0.60,
                    1.0
                ),
                1.0
            )

            area_score = min(
                area /
                max(
                    width * height * 0.03,
                    1.0
                ),
                1.0
            )


            # Prefer:
            #
            #   lower component
            #   vertically extended component
            #   sufficiently large component

            score = (
                0.45 * bottom_score
                +
                0.35 * vertical_score
                +
                0.20 * area_score
            )


            if score > line_score:

                line_score = score
                line_contour = contour


        # =====================================================================
        # CALCULATE LINE POSITION
        # =====================================================================

        line_valid = (
            line_contour is not None
        )

        line_cx = None
        line_error = 0.0


        if line_valid:

            x, y, w, h = cv2.boundingRect(
                line_contour
            )

            moments = cv2.moments(
                line_contour
            )


            if moments["m00"] > 0:

                line_cx = int(
                    round(
                        moments["m10"] /
                        moments["m00"]
                    )
                )

            else:

                line_cx = (
                    x +
                    (w // 2)
                )


            line_cx = max(
                0,
                min(
                    width - 1,
                    line_cx
                )
            )


            camera_center = (
                width // 2
            )

            line_error = float(
                line_cx -
                camera_center
            )

        # =====================================================================
        # PRIMARY AI OBJECT DETECTOR
        # =====================================================================

        (
            obstacle_detected,
            obstacle_cx,
            obstacle_area,
            obstacle_direction
        ) = self.detect_objects(
            small_frame,
            line_cx
        )


        # =====================================================================
        # OBSTACLE CONFIRMATION
        # =====================================================================

        if obstacle_detected:

            self.obstacle_confirm_count += 1
            
            print("OBSTACLE FRAME | "
              f"type={self.detected_object_type} | "
              f"conf={self.detected_object_confidence:.2f} | "
              f"cx={obstacle_cx} | "
              f"confirm={self.obstacle_confirm_count}/"
              f"{self.obstacle_confirm_required}" )

        else:

            self.obstacle_confirm_count = max(
                0,
                self.obstacle_confirm_count - 1
            )


        # =====================================================================
        # START OBSTACLE AVOIDANCE
        # =====================================================================

        if (
            not self.avoidance_active
            and
            not self.reacquire_active
            and
            self.obstacle_confirm_count
            >= self.obstacle_confirm_required
        ):

            camera_center = (
                width // 2
            )


            # Obstacle LEFT
            # Vehicle passes on RIGHT.

            if obstacle_cx < camera_center:

                self.avoidance_direction = "RIGHT"

            else:

                self.avoidance_direction = "LEFT"


            # Preserve this direction for post-obstacle recovery.
            self.recovery_direction = (
                self.avoidance_direction
            )


            self.avoidance_active = True

            self.avoidance_start_time = now

            self.obstacle_confirm_count = 0


            print(
                "OBSTACLE CONFIRMED | "
                f"direction={self.avoidance_direction} | "
                f"center={obstacle_cx} | "
                f"area={obstacle_area}"
            )


        # =====================================================================
        # OBSTACLE AVOIDANCE
        # =====================================================================

        if self.avoidance_active:

            elapsed = (
                now -
                self.avoidance_start_time
            )

            # -----------------------------------------------------------------
            # SOFT CLOSED-LOOP AVOIDANCE
            # -----------------------------------------------------------------
            #
            # Keep the normal line-following controller active.
            # Add only a small avoidance bias.
            #
            # This prevents aggressive steering reversals.
            # -----------------------------------------------------------------

            if elapsed < self.avoidance_duration:

                if self.avoidance_direction == "RIGHT":

                    # Strong RIGHT maneuver.
                    # 90 = straight.
                    # >90 = RIGHT.
                    target_angle = 108

                else:

                    # Strong LEFT maneuver.
                    # <90 = LEFT.
                    target_angle = 72


                self.state = "AVOID"

                cv2.putText(
                    small_frame,
                    (
                        f"AVOID "
                        f"{self.avoidance_direction} "
                        f"{elapsed:.1f}s"
                    ),
                    (5, 70),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.36,
                    (0, 0, 255),
                    1
                )

                cv2.putText(
                    small_frame,
                    f"AVOID ANGLE={target_angle}",
                    (5, 92),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.34,
                    (0, 165, 255),
                    1
                )

                return (
                    target_angle,
                    "DRIVE",
                    small_frame
                )

            # -----------------------------------------------------------------
            # AVOIDANCE COMPLETE
            # -----------------------------------------------------------------

            self.avoidance_active = False

            self.avoidance_direction = "NONE"

            self.reacquire_active = True

            self.reacquire_start_time = now

            self.state = "REACQUIRE"


        # =====================================================================
        # POST-OBSTACLE LINE REACQUISITION
        # =====================================================================

        if self.reacquire_active:

            elapsed = (
                now -
                self.reacquire_start_time
            )


            # -----------------------------------------------------------------
            # LINE FOUND AGAIN
            # -----------------------------------------------------------------

            if (
                line_valid
                and
                line_cx is not None
            ):

                self.reacquire_active = False

                self.state = "DRIVE"

                self.line_missed_frames = 0


                # Return DIRECTLY to closed-loop line following.

                target_angle = (
                    self.map_error_to_angle(
                        line_error
                    )
                )

                self.last_valid_angle = (
                    target_angle
                )


                cv2.circle(
                    small_frame,
                    (
                        line_cx, height //2
                    ),
                    3,
                    (0, 255, 0),
                    -1
                )


                cv2.putText(
                    small_frame,
                    "LINE REACQUIRED",
                    (5, 70),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.36,
                    (0, 255, 0),
                    1
                )


                return (
                    target_angle,
                    "DRIVE",
                    small_frame
                )


            # -----------------------------------------------------------------
            # LINE NOT YET FOUND
            # -----------------------------------------------------------------
            #
            # Continue moving slowly with a small recovery bias.
            #
            # This is intentionally much smaller than the main avoidance
            # steering command.
            # -----------------------------------------------------------------

            if elapsed < self.reacquire_timeout:

                if (
                    self.recovery_direction
                    ==
                    "RIGHT"
                ):

                    search_angle = 87

                elif (
                    self.recovery_direction
                    ==
                    "LEFT"
                ):

                    search_angle = 93

                else:

                    search_angle = 90


                self.state = "REACQUIRE"


                cv2.putText(
                    small_frame,
                    (
                        f"REACQUIRE "
                        f"{self.reacquire_timeout - elapsed:.1f}s"
                    ),
                    (5, 70),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.36,
                    (0, 255, 255),
                    1
                )


                return (
                    search_angle,
                    "DRIVE",
                    small_frame
                )


            # -----------------------------------------------------------------
            # REACQUISITION FAILED
            # -----------------------------------------------------------------

            self.reacquire_active = False

            self.state = "SEARCH_STOP"

            self.lost_line_timestamp = now


        # =====================================================================
        # NORMAL YELLOW LINE FOLLOWING
        # =====================================================================

        if (
            line_valid
            and
            line_cx is not None
        ):

            self.line_missed_frames = 0

            self.state = "DRIVE"


            target_angle = (
                self.map_error_to_angle(
                    line_error
                )
            )


            self.last_valid_angle = (
                target_angle
            )


            camera_center = (
                width // 2
            )


            # -----------------------------------------------------------------
            # Display line centre
            # -----------------------------------------------------------------

            cv2.circle(
                small_frame,(line_cx, height // 2),
                3,
                (0, 255, 0),
                -1
            )


            # -----------------------------------------------------------------
            # Display camera centre
            # -----------------------------------------------------------------

            cv2.line(
                small_frame,
                (
                    camera_center,
                    roi_start
                ),
                (
                    camera_center,
                    height - 1
                ),
                (255, 0, 0),
                1
            )


            # -----------------------------------------------------------------
            # Diagnostics
            # -----------------------------------------------------------------

            cv2.putText(
                small_frame,
                (
                    f"LINE "
                    f"E={int(line_error)} "
                    f"A={target_angle}"
                ),
                (5, 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.36,
                (0, 255, 0),
                1
            )


            cv2.putText(
                small_frame,
                (
                    f"DRIVE "
                    f"Y={cv2.countNonZero(roi_mask)}"
                ),
                (5, 42),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.34,
                (0, 255, 0),
                1
            )


            return (
                target_angle,
                "DRIVE",
                small_frame
            )


        # =====================================================================
        # YELLOW LINE LOST
        # =====================================================================

        self.line_missed_frames += 1

        target_angle = self.servo_center

        engine_state = "STOP"


        # ---------------------------------------------------------------------
        # First line-loss event
        # ---------------------------------------------------------------------

        if self.state == "DRIVE":

            self.state = "SEARCH_STOP"

            self.lost_line_timestamp = now


        # ---------------------------------------------------------------------
        # SEARCH STOP
        # ---------------------------------------------------------------------

        if self.state == "SEARCH_STOP":

            elapsed = (
                now -
                self.lost_line_timestamp
            )


            if elapsed >= 3.5:

                self.state = "REVERSE"

                self.reverse_start_timestamp = now


        # ---------------------------------------------------------------------
        # REVERSE RECOVERY
        # ---------------------------------------------------------------------

        if self.state == "REVERSE":

            elapsed_reverse = (
                now -
                self.reverse_start_timestamp
            )


            if elapsed_reverse < 3.0:

                reverse_angle = (
                    180 -
                    self.last_valid_angle
                )


                target_angle = max(
                    45,
                    min(
                        135,
                        int(reverse_angle)
                    )
                )


                engine_state = "REVERSE"


            else:

                target_angle = (
                    self.servo_center
                )

                engine_state = "STOP"


        # =====================================================================
        # FAILSAFE DISPLAY
        # =====================================================================

        cv2.putText(
            small_frame,
            f"FAILSAFE: {self.state}",
            (5, 70),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.36,
            (0, 0, 255),
            1
        )


        cv2.putText(
            small_frame,
            (
                f"LINE LOST "
                f"{self.line_missed_frames}"
            ),
            (5, 92),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.34,
            (0, 255, 255),
            1
        )


        # =====================================================================
        # FINAL RETURN
        # =====================================================================

        return (
            target_angle,
            engine_state,
            small_frame
        )