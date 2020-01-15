unsigned long initial_beeps_time_now, initial_beeps_time_elapsed, initial_beeps_timestamp;
unsigned long approach_pid_time_now, approach_pid_time_elapsed, approach_pid_timestamp;
unsigned long found_line_beeps_time_now, found_line_beeps_time_elapsed, found_line_beeps_timestamp;
unsigned long follow_line_pid_time_now, follow_line_pid_time_elapsed, follow_line_pid_timestamp;
unsigned long general_find_line_time_now, general_find_line_time_elapsed, general_find_line_timestamp;
unsigned long turn_to_theta_pid_time_now, turn_to_theta_pid_time_elapsed, turn_to_theta_pid_timestamp;
unsigned long drive_home_pid_time_now, drive_home_pid_time_elapsed, drive_home_pid_timestamp;

void initialise_timer_variables() {
    initial_beeps_time_now     = millis();
    initial_beeps_time_elapsed = 0;
    initial_beeps_timestamp    = 0;

    approach_pid_time_now     = millis();
    approach_pid_time_elapsed = 0;
    approach_pid_timestamp    = 0;

    found_line_beeps_time_now     = millis();
    found_line_beeps_time_elapsed = 0;
    found_line_beeps_timestamp    = 0;

    follow_line_pid_time_now     = millis();
    follow_line_pid_time_elapsed = 0;
    follow_line_pid_timestamp    = 0;

    general_find_line_time_now     = millis();
    general_find_line_time_elapsed = 0;
    general_find_line_timestamp    = 0;

    turn_to_theta_pid_time_now     = millis();
    turn_to_theta_pid_time_elapsed = 0;
    turn_to_theta_pid_timestamp    = 0;

    drive_home_pid_time_now     = millis();
    drive_home_pid_time_elapsed = 0;
    drive_home_pid_timestamp    = 0;

}