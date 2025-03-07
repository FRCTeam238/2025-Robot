/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;


/**
 * Add your docs here.
 */
public class DataFileAutonomousModeDataSource implements IAutonomousModeDataSource {
    private String filePath;

    public DataFileAutonomousModeDataSource(String filePath) {
        // should add null or whitespace check
        this.filePath = filePath;
    }

    @Override
    public String getJson() {
        try {
            return Files.readString(Paths.get(filePath));
        } catch (IOException e) {
            System.out.println("DataFileAutonomousModeDataSource.GetJson failed : " + filePath);
            return null;
        }
    }

}
