// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Do NOT add any static variables to this class, or any initialization at all. Unless you know what
 * you are doing, do not modify this file except to change the parameter class to the startRobot
 * call.
 */
public final class Main {
    private Main() {
    }

    /**
     * Main initialization function. Do not perform any initialization here.
     *
     * <p>If you change your main robot class, change the parameter type.
     */
    public static void main(String... args) {
        System.out.println("\n\n    ____            __             _     __            __         ");
        System.out.println("   / __ \\  ____ _  / /_   _____   (_)   / /_   ____   / /_   _____");
        System.out.println("  / /_/ / / __ `/ / __/  / ___/  / /   / __ \\ / __ \\ / __/  / ___/");
        System.out.println(" / ____/ / /_/ / / /_   / /     / /   / /_/ // /_/ // /_   (__  ) ");
        System.out.println("/_/      \\__,_/  \\__/  /_/     /_/   /_.___/ \\____/ \\__/  /____/  ");
        System.out.println("    __ __ _____   _____   ____ ");
        System.out.println("   / // //__  /  |__  /  ( __ )");
        System.out.println("  / // /_  / /    /_ <  / __  |");
        System.out.println(" /__  __/ / /   ___/ / / /_/ /");
        System.out.println("   /_/   /_/   /____/  \\____/\n");
        RobotBase.startRobot(Robot::new);
    }
}
