package org.firstinspires.ftc.vision.apriltag;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;

public class AprilTagGameDatabase
{
    /**
     * Get the {@link AprilTagLibrary} for the current season game, plus sample tags
     * @return the {@link AprilTagLibrary} for the current season game, plus sample tags
     */
    public static AprilTagLibrary getCurrentGameTagLibrary()
    {
        return new AprilTagLibrary.Builder()
                .addTags(getSampleTagLibrary())
                .addTags(getDecodeTagLibrary())
                .build();
    }

    /**
     * Get the {@link AprilTagLibrary} for the Center Stage FTC game
     * @return the {@link AprilTagLibrary} for the Center Stage FTC game
     */
    public static AprilTagLibrary getCenterStageTagLibrary()
    {
        return new AprilTagLibrary.Builder()
                .addTag(1, "BlueAllianceLeft",
                        2, new VectorF(60.25f, 41.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3535534f, -0.6123724f, 0.6123724f, -0.3535534f, 0))
                .addTag(2, "BlueAllianceCenter",
                        2, new VectorF(60.25f, 35.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3535534f, -0.6123724f, 0.6123724f, -0.3535534f, 0))
                .addTag(3, "BlueAllianceRight",
                        2, new VectorF(60.25f, 29.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3535534f, -0.6123724f, 0.6123724f, -0.3535534f, 0))
                .addTag(4, "RedAllianceLeft",
                        2, new VectorF(60.25f, -29.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3535534f, -0.6123724f, 0.6123724f, -0.3535534f, 0))
                .addTag(5, "RedAllianceCenter",
                        2, new VectorF(60.25f, -35.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3535534f, -0.6123724f, 0.6123724f, -0.3535534f, 0))
                .addTag(6, "RedAllianceRight",
                        2, new VectorF(60.25f, -41.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3535534f, -0.6123724f, 0.6123724f, -0.3535534f, 0))
                .addTag(7, "RedAudienceWallLarge",
                        5, new VectorF(-70.25f, -40.625f, 5.5f), DistanceUnit.INCH,
                        new Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0))
                .addTag(8, "RedAudienceWallSmall",
                        2, new VectorF(-70.25f, -35.125f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0))
                .addTag(9, "BlueAudienceWallSmall",
                        2, new VectorF(-70.25f, 35.125f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0))
                .addTag(10, "BlueAudienceWallLarge",
                        5, new VectorF(-70.25f, 40.625f, 5.5f), DistanceUnit.INCH,
                        new Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0))
                .build();
    }

    /**
     * Get the {@link AprilTagLibrary} for the Into The Deep FTC game
     * @return the {@link AprilTagLibrary} for the Into The Deep FTC game
     */
    public static AprilTagLibrary getIntoTheDeepTagLibrary()
    {
        return new AprilTagLibrary.Builder()
                .addTag(11, "BlueAudienceWall",
                        4, new VectorF(-70.25f, 46.83f, 5.75f), DistanceUnit.INCH,
                        new Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0))
                .addTag(12, "BlueAllianceWall",
                        4, new VectorF(0.0f, 70.25f, 5.75f), DistanceUnit.INCH,
                        new Quaternion(0.7071067f, -0.7071067f, 0.0f, 0.0f, 0))
                .addTag(13, "BlueRearWall",
                        4, new VectorF(70.25f, 46.83f, 5.75f), DistanceUnit.INCH,
                        new Quaternion(0.5f, -0.5f, 0.5f, -0.5f, 0))
                .addTag(14, "RedRearWall",
                        4, new VectorF(70.25f, -46.83f, 5.75f), DistanceUnit.INCH,
                        new Quaternion(0.5f, -0.5f, 0.5f, -0.5f, 0))
                .addTag(15, "RedAllianceWall",
                        4, new VectorF(0.0f, -70.25f, 5.75f), DistanceUnit.INCH,
                        new Quaternion(0.0f, 0.0f, 0.7071067f, -0.7071067f, 0))
                .addTag(16, "RedAudienceWall",
                        4, new VectorF(-70.25f, -46.83f, 5.75f), DistanceUnit.INCH,
                        new Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0))
                .build();
    }

    /**
     * Get the {@link AprilTagLibrary} for the Decode FTC game
     * @return the {@link AprilTagLibrary} for the Decode FTC game
     */
    public static AprilTagLibrary getDecodeTagLibrary(){
        return new AprilTagLibrary.Builder()
                .addTag(20, "BlueTarget",
                        6.5, new VectorF(-58.3727f, -55.6425f, 29.5f), DistanceUnit.INCH,
                        new Quaternion(0.2182149f, -0.2182149f, -0.6725937f, 0.6725937f, 0))
                .addTag(21, "Obelisk_GPP",
                        6.5, DistanceUnit.INCH)
                .addTag(22, "Obelisk_PGP",
                        6.5, DistanceUnit.INCH)
                .addTag(23, "Obelisk_PPG",
                        6.5, DistanceUnit.INCH)
                .addTag(24, "RedTarget",
                        6.5, new VectorF(-58.3727f, 55.6425f, 29.5f), DistanceUnit.INCH,
                        new Quaternion(0.6725937f, -0.6725937f, -0.2182149f, 0.2182149f, 0))
                .build();
    }

    /**
     * Get the {@link AprilTagLibrary} for the tags used in the sample OpModes
     * @return the {@link AprilTagLibrary} for the tags used in the sample OpModes
     */
    public static AprilTagLibrary getSampleTagLibrary()
    {
        return new AprilTagLibrary.Builder()
                .addTag(583, "Nemo",
                        4, DistanceUnit.INCH)
                .addTag(584, "Jonah",
                        4, DistanceUnit.INCH)
                .addTag(585, "Cousteau",
                        6, DistanceUnit.INCH)
                .addTag(586, "Ariel",
                        6, DistanceUnit.INCH)
                .build();
    }
}

