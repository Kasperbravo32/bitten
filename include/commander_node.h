#pragma once

struct Waypoint_s {
    double jointPositions[6];
    double velocity[6];
    double effort[6];
};

enum INPUT_MODE {
    MANUAL_MODE,
    WP_MODE,
    TEST_MODE,
    POLL_MODE,
    NO_MODE
    
} INPUT_MODE;


 /* ----------------------------------------------------------------------
 *                      -------  Initializing   -------
 * ----------------------------------------------------------------------- */
void manualCallback             (const bitten::control_msg::ConstPtr& manual        );
void wpCallback                 (const bitten::control_msg::ConstPtr& wp            );
void terminalCallback           (const bitten::control_msg::ConstPtr& terminal      );
void movementFeedbackCallback   (const bitten::feedback_msg::ConstPtr& moveFeedback );


