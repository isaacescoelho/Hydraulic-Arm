import os
import sys
from robot_arm import RoboticArm

BLUE = "\033[94m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
RED = "\033[91m"
CYAN = "\033[96m"
MAGENTA = "\033[95m"
BOLD = "\033[1m"
RESET = "\033[0m"

HEADER_WIDTH = 50

# Functions to make the code less messy


def clear_screen():
    print("\033[2J\033[H", end="")


def print_arm_dimensions(arm):
    print(f"{GREEN}{BOLD}Robot Dimensions{RESET}")
    print(f" Arm 1 Length : {arm.arm_1_length:.2f} cm")
    print(f" Arm 2 Length : {arm.arm_2_length:.2f} cm")
    print(f" Claw Length  : {arm.claw_length:.2f} cm")
    print()
    print(f" Piston 1's distances from arm: {arm.piston_1_distance:.2f} cm {arm.piston_1_end_distance:.2f} cm")
    print(f" Piston 2's distances from arm: {arm.piston_2_distance:.2f} cm {arm.piston_2_end_distance:.2f} cm")
    print(f" Piston 3's distances from arm: {arm.piston_3_distance:.2f} cm {arm.piston_3_end_distance:.2f} cm")
    print()


def draw_header():
    clear_screen()

    print(f"{CYAN}{BOLD}{'=' * HEADER_WIDTH}")
    print("        WELCOME TO THE ROBOTIC ARM CONTROLLER        ")
    print(f"{'=' * HEADER_WIDTH}{RESET}\n")


def pause():
    input(f"\n{YELLOW}Press Enter to continue...{RESET}")
    clear_screen()


def pause_error():
    input(f"\n{RED}Press Enter to continue...{RESET}")
    clear_screen()


def get_input(prompt):
    while True:
        user_input = input(
            f"{YELLOW}{prompt}: {RESET}"
        ).strip()

        try:
            return float(user_input)

        except ValueError:
            print(
                f"{RED}Invalid input. "
                f"Please enter a numeric value.{RESET}"
            )


def print_menu():
    print(f" [{BLUE}1{RESET}] Find Piston Lengths for Coordinates, 3D")
    print(f" [{BLUE}2{RESET}] Find Coordinates with Piston Lengths, 3D")
    print(f" [{BLUE}3{RESET}] Find Piston Lengths for Coordinates, 2D")
    print(f" [{BLUE}4{RESET}] Find Coordinates with Piston Lengths, 2D")
    print(f" [{BLUE}5{RESET}] Edit Arm and Piston Dimensions")
    print(f" [{BLUE}6{RESET}] Exit")


# ---------------------------------------------------------------------------

def main():
    arm = RoboticArm()

    while True:
        draw_header()
        print_arm_dimensions(arm)
        print_menu()

        choice = input(
            f"\n{YELLOW}Select an option: {RESET}"
        ).strip()

        if choice == "1":
            print(f"\n{CYAN}{BOLD}--- 3D PISTON LENGTH FINDER ---{RESET}")

            x = get_input("Enter X Coordinate (cm)")
            y = get_input("Enter Y Coordinate (cm)")
            z = get_input("Enter Z Coordinate (cm)")
            angle = get_input("Enter Claw Angle")

            try:
                # Inverse kinematics turns coordinates into piston lengths

                base_deg, p1, p2, p3 = arm.inverse_three_dimensional(
                    x,
                    y,
                    z,
                    angle,
                )

                print(f"\n{MAGENTA}{BOLD}[Piston Lengths and Rotation]{RESET}")
                print(f" -> Base Rotation : {BLUE}{base_deg:.2f}°{RESET}")
                print(f" -> Piston 1 Ext  : {BLUE}{p1:.4f}{RESET} cm")
                print(f" -> Piston 2 Ext  : {BLUE}{p2:.4f}{RESET} cm")
                print(f" -> Piston 3 Ext  : {BLUE}{p3:.4f}{RESET} cm")

                pause()

            except ValueError as error:
                print(
                    f"\n{RED}{BOLD}"
                    f"Execution Error: {error}"
                    f"{RESET}"
                )

                pause_error()

        elif choice == "2":
            print(f"\n{CYAN}{BOLD}--- 3D FIND COORDINATES ---{RESET}")

            base_deg = get_input("Enter Base Rotation")
            p1 = get_input("Enter Piston 1 Length (cm)")
            p2 = get_input("Enter Piston 2 Length (cm)")
            p3 = get_input("Enter Piston 3 Length (cm)")

            # Forward kinematics turns piston lengths into coordinates

            fk = arm.forward_kinematics_details(
                base_deg,
                p1,
                p2,
                p3,
            )

            print(f"\n{GREEN}{BOLD}[Joint Angles]{RESET}")
            print(f" -> Joint 1 : {fk['joint_1_deg']:.2f}°")
            print(f" -> Joint 2 : {fk['joint_2_deg']:.2f}°")
            print(f" -> Joint 3 : {fk['joint_3_deg']:.2f}°")

            print(f"\n{GREEN}{BOLD}[Relative Angles]{RESET}")
            print(
                f" -> Joint 1 Relative : "
                f"{fk['relative_joint_1_deg']:.2f}°"
            )
            print(
                f" -> Joint 2 Relative : "
                f"{fk['relative_joint_2_deg']:.2f}°"
            )
            print(
                f" -> Joint 3 Relative : "
                f"{fk['relative_joint_3_deg']:.2f}°"
            )

            print(f"\n{GREEN}{BOLD}[3D Position]{RESET}")
            print(f" -> X : {fk['x']:.4f} cm")
            print(f" -> Y : {fk['y']:.4f} cm")
            print(f" -> Z : {fk['z']:.4f} cm")

            pause()

        elif choice == "3":
            print(f"\n{CYAN}{BOLD}--- 2D PISTON LENGTH FINDER ---{RESET}")

            x = get_input("Enter X Coordinate (cm)")
            y = get_input("Enter Y Coordinate (cm)")
            angle = get_input("Enter Claw Angle (deg)")

            try:
                p1, p2, p3 = arm.inverse_two_dimensional(
                    x,
                    y,
                    angle,
                )

                print(f"\n{MAGENTA}{BOLD}[Piston Lengths]{RESET}")
                print(f" -> Piston 1 Ext : {BLUE}{p1:.4f} cm{RESET}")
                print(f" -> Piston 2 Ext : {BLUE}{p2:.4f} cm{RESET}")
                print(f" -> Piston 3 Ext : {BLUE}{p3:.4f} cm{RESET}")

                pause()

            except ValueError as error:
                print(
                    f"\n{RED}{BOLD}"
                    f"Execution Error: {error}"
                    f"{RESET}"
                )

                pause_error()

        elif choice == "4":
            print(f"\n{CYAN}{BOLD}--- 2D FIND COORDINATES ---{RESET}")

            p1 = get_input("Enter Piston 1 Length (cm)")
            p2 = get_input("Enter Piston 2 Length (cm)")
            p3 = get_input("Enter Piston 3 Length (cm)")

            fk = arm.forward_kinematics_details(
                0.0,
                p1,
                p2,
                p3,
            )

            print(f"\n{GREEN}{BOLD}[Joint Angles]{RESET}")
            print(f" -> Joint 1 : {fk['joint_1_deg']:.2f}°")
            print(f" -> Joint 2 : {fk['joint_2_deg']:.2f}°")
            print(f" -> Joint 3 : {fk['joint_3_deg']:.2f}°")

            print(f"\n{GREEN}{BOLD}[2D Position]{RESET}")
            print(f" -> Radius : {fk['radius']:.4f} cm")
            print(f" -> Height : {fk['height']:.4f} cm")

            pause()

        elif choice == "5":
            print(f"\n{CYAN}{BOLD}--- EDIT ARM DIMENSIONS ---{RESET}")

            try:
                print("Press Enter to keep current value.\n")

                arm1 = input("Arm 1 Length (cm): ").strip()
                arm2 = input("Arm 2 Length (cm): ").strip()
                claw = input("Claw Length (cm): ").strip()
                print()
                piston1 = input("Piston 1 Distances [space separated] (cm): ").split()
                piston1 = [float(x.strip()) for x in piston1]
                piston2 = input("Piston 2 Distances [space separated] (cm): ").split()
                piston2 = [float(x.strip()) for x in piston2]
                piston3 = input("Piston 3 Distances [space separated] (cm): ").split()
                piston3 = [float(x.strip()) for x in piston3]

                arm.set_arm_dimensions(
                    arm1=float(arm1) if arm1 else None,
                    arm2=float(arm2) if arm2 else None,
                    claw=float(claw) if claw else None,
                    piston1a=piston1[0],
                    piston1b=piston1[1],
                    piston2a=piston2[0],
                    piston2b=piston2[1],
                    piston3a=piston3[0],
                    piston3b=piston3[1],
                )

                print(f"\n{GREEN}Arm and piston dimensions changed!{RESET}")
                pause()

            except ValueError as error:
                print(f"\n{RED}{BOLD}Error: {error}{RESET}")
                pause_error()

        elif choice == "6":
            clear_screen()
            print(f"{GREEN}Goodbye!{RESET}")
            sys.exit()

        else:
            print(
                f"{RED}Invalid selection. "
                f"Press Enter to try again.{RESET}"
            )

            pause_error()


# Code I copied to make colors work on windows

if __name__ == "__main__":
    if os.name == "nt":
        os.system("color")

    main()