Universe = vanilla

#State that we are an undergrad. 
+Group = "UNDER"

#We will almost always be submitting something AI related. 
+Project = "AI_ROBOTICS"

+ProjectDescription = "Speech & Semantic Parsing Re-ranking Experiment"

#Will run the GPU job in our virtual environment. 
executable = /bin/bash
arguments = run_experiments.bash $(data_percent) $(starting_parser_path)

#Condor job log file. 
log = $(log)

#Redirect standard out and error to these files. 
output = $(output)
error = $(error)

#We want to know when the job finishes.
Notification = Complete

#Queue the job. 
Queue 1
