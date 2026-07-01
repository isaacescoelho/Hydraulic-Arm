import math


class RoboticArm:

    def __init__(self):
        # Fixed distances for the piston mounting geometry

        self.piston_1_distance = 5.0
        self.piston_1_end_distance = 2.0

        self.piston_2_distance = 5.0
        self.piston_2_end_distance = 2.0

        self.piston_3_distance = 5.0
        self.piston_3_end_distance = 2.0

        # Lengths of the arm sections

        self.arm_1_length = 20.0
        self.arm_2_length = 20.0
        self.claw_length = 7.0

    def set_arm_dimensions(self, arm1=None, arm2=None, claw=None, piston1a=None, piston1b=None, piston2a=None, piston2b=None, piston3a=None, piston3b=None):
        if arm1 is not None:
            if arm1 <= 0:
                raise ValueError("Arm 1 length must be positive.")
            self.arm_1_length = arm1

        if arm2 is not None:
            if arm2 <= 0:
                raise ValueError("Arm 2 length must be positive.")
            self.arm_2_length = arm2

        if claw is not None:
            if claw <= 0:
                raise ValueError("Claw length must be positive.")
            self.claw_length = claw

        if piston1a is not None:
            if piston1a <= 0:
                raise ValueError("Piston 1 distances must both be positive.")
            self.piston_1_distance = piston1a
        if piston1b is not None:
            if piston1b <= 0:
                raise ValueError("Piston 1 distances must both be positive.")
            self.piston_1_end_distance = piston1b

        if piston2a is not None:
            if piston2a <= 0:
                raise ValueError("Piston 2 distances must both be positive.")
            self.piston_2_distance = piston2a
        if piston2b is not None:
            if piston2b <= 0:
                raise ValueError("Piston 2 distances must both be positive.")
            self.piston_2_end_distance = piston2b

        if piston3a is not None:
            if piston3a <= 0:
                raise ValueError("Piston 3 distances must both be positive.")
            self.piston_3_distance = piston3a
        if piston3b is not None:
            if piston3b <= 0:
                raise ValueError("Piston 3 distances must both be positive.")
            self.piston_3_end_distance = piston3b

    def _calculate_joint_angle(self, distance, end_distance, length):
        # Because I use this so much it is a function, that uses the law of cosines

        ratio = (
            distance**2
            + end_distance**2
            - length**2
        ) / (2 * distance * end_distance)

        # The acos has to have values -1 < x < 1 so I clamp it

        ratio = max(-1.0, min(1.0, ratio))

        return math.acos(ratio)

    def _get_piston_length(
        self,
        distance,
        end_distance,
        joint_angle,
    ):

        # Uses the law of cosines backwards to get the piston length

        return math.sqrt(
            distance**2
            + end_distance**2
            - 2
            * distance
            * end_distance
            * math.cos(abs(joint_angle))
        )

    def _relative_angles_from_pistons(
        self,
        piston_length_1,
        piston_length_2,
        piston_length_3,
    ):
        rel_angle_1 = self._calculate_joint_angle(
            self.piston_1_distance,
            self.piston_1_end_distance,
            piston_length_1,
        )

        # These joints rotate the opposite direction on my arm

        rel_angle_2 = -self._calculate_joint_angle(
            self.piston_2_distance,
            self.piston_2_end_distance,
            piston_length_2,
        )

        rel_angle_3 = -self._calculate_joint_angle(
            self.piston_3_distance,
            self.piston_3_end_distance,
            piston_length_3,
        )

        return rel_angle_1, rel_angle_2, rel_angle_3

    def forward_kinematics_details(
        self,
        base_rotation_deg,
        piston_length_1,
        piston_length_2,
        piston_length_3,
    ):

        rel_angle_1, rel_angle_2, rel_angle_3 = (
            self._relative_angles_from_pistons(
                piston_length_1,
                piston_length_2,
                piston_length_3,
            )
        )

        # Relative angles are between links, actual angles are from horizontal

        angle_1 = rel_angle_1
        angle_2 = angle_1 + rel_angle_2
        angle_3 = angle_2 + rel_angle_3

        radius = (
            math.cos(angle_1) * self.arm_1_length
            + math.cos(angle_2) * self.arm_2_length
            + math.cos(angle_3) * self.claw_length
        )

        height = (
            math.sin(angle_1) * self.arm_1_length
            + math.sin(angle_2) * self.arm_2_length
            + math.sin(angle_3) * self.claw_length
        )

        base_rotation_rad = math.radians(base_rotation_deg)

        # Rotates the 2D arm position around the base

        x = radius * math.cos(base_rotation_rad)
        y = radius * math.sin(base_rotation_rad)

        return {
            "joint_1_deg": math.degrees(angle_1),
            "joint_2_deg": math.degrees(angle_2),
            "joint_3_deg": math.degrees(angle_3),
            "relative_joint_1_deg": math.degrees(rel_angle_1),
            "relative_joint_2_deg": math.degrees(rel_angle_2),
            "relative_joint_3_deg": math.degrees(rel_angle_3),
            "radius": radius,
            "height": height,
            "x": x,
            "y": y,
            "z": height,
        }

    def two_dimensional(
        self,
        piston_length_1,
        piston_length_2,
        piston_length_3,
    ):
        fk = self.forward_kinematics_details(
            base_rotation_deg=0.0,
            piston_length_1=piston_length_1,
            piston_length_2=piston_length_2,
            piston_length_3=piston_length_3,
        )

        return [fk["radius"], fk["height"]]

    def three_dimensional(
        self,
        base_rotation_deg,
        piston_length_1,
        piston_length_2,
        piston_length_3,
    ):
        fk = self.forward_kinematics_details(
            base_rotation_deg,
            piston_length_1,
            piston_length_2,
            piston_length_3,
        )

        return [fk["x"], fk["y"], fk["z"]]

    def inverse_two_dimensional(
        self,
        radius,
        height,
        claw_angle_deg,
    ):
        claw_angle_rad = math.radians(claw_angle_deg)

        # Removes the claw length so I can solve for the wrist first

        wrist_radius = (
            radius
            - self.claw_length * math.cos(claw_angle_rad)
        )

        wrist_height = (
            height
            - self.claw_length * math.sin(claw_angle_rad)
        )

        distance_squared = (
            wrist_radius**2 + wrist_height**2
        )

        distance = math.sqrt(distance_squared)

        max_reach = self.arm_1_length + self.arm_2_length
        min_reach = abs(
            self.arm_1_length - self.arm_2_length
        )

        # Checks if the target is physically reachable

        if distance > max_reach or distance < min_reach:
            raise ValueError(
                f"Target coordinate out of reach. "
                f"Radial reach: {distance:.2f}"
            )

        cos_beta = (
            self.arm_1_length**2
            + self.arm_2_length**2
            - distance_squared
        ) / (
            2
            * self.arm_1_length
            * self.arm_2_length
        )

        # Uses the law of cosines to find the elbow angle

        beta = math.acos(max(-1.0, min(1.0, cos_beta)))

        # Negative because this joint bends the opposite direction

        angle_2_relative = -(math.pi - beta)

        alpha_1 = math.atan2(
            wrist_height,
            wrist_radius,
        )

        cos_alpha_2 = (
            self.arm_1_length**2
            + distance_squared
            - self.arm_2_length**2
        ) / (
            2
            * self.arm_1_length
            * distance
        )

        # Gets the extra shoulder angle needed

        alpha_2 = math.acos(
            max(-1.0, min(1.0, cos_alpha_2))
        )

        angle_1 = alpha_1 + alpha_2

        # Makes the claw point where I want it

        angle_3_relative = (
            claw_angle_rad
            - (angle_1 + angle_2_relative)
        )

        piston_1 = self._get_piston_length(
            self.piston_1_distance,
            self.piston_1_end_distance,
            angle_1,
        )

        piston_2 = self._get_piston_length(
            self.piston_2_distance,
            self.piston_2_end_distance,
            angle_2_relative,
        )

        piston_3 = self._get_piston_length(
            self.piston_3_distance,
            self.piston_3_end_distance,
            angle_3_relative,
        )

        return piston_1, piston_2, piston_3

    def inverse_three_dimensional(
        self,
        x,
        y,
        z,
        claw_angle_deg,
    ):
        # Gets the base rotation from x and y

        base_rotation_rad = math.atan2(y, x)
        base_rotation_deg = math.degrees(base_rotation_rad)

        # Converts x and y into the radius used by the 2D solver

        radius = math.sqrt(x**2 + y**2)

        piston_1, piston_2, piston_3 = (
            self.inverse_two_dimensional(
                radius,
                z,
                claw_angle_deg,
            )
        )

        return (
            base_rotation_deg,
            piston_1,
            piston_2,
            piston_3,
        )
