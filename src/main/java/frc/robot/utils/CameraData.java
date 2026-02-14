// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;

/** Add your docs here. */
public class CameraData {
    public boolean limeLightExists;
    public boolean isActive;
    public boolean rejectMT1Update;
    public boolean rejectMT2Update;
    public boolean m_useMegaTag2;

    public boolean setMT2toMT1Rotation;

    public boolean inhibitVision;

    public double MT1distToCamera;
    public double MT1ambiguity;

    public double MT2distToCamera;
    public double MT2ambiguity;

    public int numberMT2Pose;
    public int MT1tagCount;

    public Pose2d mt1Pose = new Pose2d();
    public Pose2d mt2Pose = new Pose2d();
    public boolean orientationSet;

    public CameraData(String cameraName) {

    }

}
