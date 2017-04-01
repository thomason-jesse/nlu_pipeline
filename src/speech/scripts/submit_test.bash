Universe = vanilla

#State that we are an undergrad. 
+Group = "UNDER"

#We will almost always be submitting something AI related. 
+Project = "AI_ROBOTICS"

+ProjectDescription = "Speech & Semantic Parsing Re-ranking Experiment"

#Will run the GPU job in our virtual environment. 
executable = /usr/bin/python
arguments = experiments.py parse_score_google $(fold_path)/experiments/asr/test_files/test0.txt $(fold_path)/experiments/asr/result_files/$(name).test0 $(fold_path)/models/$(name).cky ../corpora/raw/google_recognized_with_names/

#Condor job log file. 
log = $(log)

#Redirect standard out and error to these files. 
output = $(output)
error = $(output)

#We want to know when the job finishes.
Notification = Complete

#Queue the job. 
Queue 1
