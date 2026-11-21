The deck: everything the operator sets on the day, in one place.

  bringup.yaml            the vehicle and the run: models, switches, nodes:.
                          `course: <name>` names the venue's course file.
  courses/<name>.yaml     the venue, ONE file: pool depth and floor survey,
                          tile size, lane lines, the depth floor, and every
                          prop's position, heading and measured width.

Nothing about the venue goes in bringup.yaml: a value set in both is refused
at launch. Edit, then check before anything moves:

  ros2 run mongla_manager bringup_check

Survey a prop into the course:  ros2 run mongla_localization course_survey --help
The full guide: .claude/context/missions/course-file.md in the repository.

~/.mongla is a link to this folder, so older paths still work.
