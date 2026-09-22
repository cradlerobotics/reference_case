  #!/bin/bash                                     
  
  echo "Killing smach_visit..."
  rosnode kill /smach_inspection_fsm 2>/dev/null

  sleep 1

  pkill -f smach_integrated.py 2>/dev/null 

  sleep 1     
  
  kill -9 $(pgrep -f smach_integrated.py) 2>/dev/null

  sleep 1

  rosnode kill -a 2>/dev/null                                                                                                                                                               
  echo "Done."  