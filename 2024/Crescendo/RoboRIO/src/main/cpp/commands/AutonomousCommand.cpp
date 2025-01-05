//==============================================================================
// AutonomousCommand.cpp
//==============================================================================

#include "AutonomousCommand.h"

#include "AutoCommands2024.h"
#include "AutoTestSwerveFollower.h"
#include "TestCharacteriseDriveBase.h"
#include "VisionTestCommand.h"

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Commands.h>

#include <vector>


namespace {
// Smart dashboard chooser for the autonomouse strategy
frc::SendableChooser<IAutonomousProvider*> ms_strategy_chooser;

// List of autonomouse providers
std::vector<IAutonomousProvider*> ms_autonomous_providers_list;
}

void AutonomousCommand::SetupGameAutonomousShuffleboard(frc::ShuffleboardTab& auto_tab, int x_pos, int y_pos) {
    // Add just the game auto command provider to the list and that is all we want
    ms_autonomous_providers_list.push_back(new AutoCommands2024());

    // Setup the game auto selecter on the provided tab
    ms_autonomous_providers_list[0]->SetupDashboard(auto_tab, x_pos, y_pos);
}

void AutonomousCommand::SetupDevAutonomousShuffleboard() {
    // Create a list of objects that provide different autonomous commands
    ms_autonomous_providers_list.push_back(new AutoCommands2024());
    ms_autonomous_providers_list.push_back(new AutoTestSwerveFollower());
    ms_autonomous_providers_list.push_back(new TestCharacteriseDriveBase::Provider());
    ms_autonomous_providers_list.push_back(new VisionTestCommand());

    // Setup the chooser for determining which autonomous provider will create the autonomous command
    for (auto & provider : ms_autonomous_providers_list) {
        ms_strategy_chooser.AddOption(provider->Name(), provider);
    }
    ms_strategy_chooser.SetDefaultOption(ms_autonomous_providers_list[0]->Name(), ms_autonomous_providers_list[0]);

    // Create a tab for the controlling autos and add the chooser for selecting the which autonomous provider
    // will create the autonomous command
    frc::ShuffleboardTab& shuffleboard_tab = frc::Shuffleboard::GetTab("Auto");
    shuffleboard_tab.Add("Auto Type", ms_strategy_chooser).WithPosition(0, 0).WithSize(6, 3);

    // Get each autonomous provider to create the shuffleboard interface it needs. Note that they
    // can add controls to the main auto tab or create their own tab.
    int x_pos = 0;
    int y_pos = 3;
    for (IAutonomousProvider* provider : ms_autonomous_providers_list) {
        x_pos += provider->SetupDashboard(shuffleboard_tab, x_pos, y_pos);
    }
}

frc2::CommandPtr AutonomousCommand::CreateAutonomousCommand(SwerveDrivebase& drivebase, Manipulator& manipulator) {
    // If there is only a single autonomous provider then use it to create the command. This
    // happens when the shuffleboard is set up for playing the game.
    if (ms_autonomous_providers_list.size() == 1) {
        return ms_autonomous_providers_list[0]->CreateAutonomousCommand(drivebase, manipulator);
    }

    // Get the autonomous provider that the user has selected on the dashboard and
    // get it to create the autonomous command
    IAutonomousProvider* provider = ms_strategy_chooser.GetSelected();
    if (provider != nullptr) {
        return provider->CreateAutonomousCommand(drivebase, manipulator);
    }

    // If no provider was selected return a command that does nothing
    return frc2::CommandPtr(frc2::InstantCommand([] {  }, {  }));
}
