requirements = InMastodon

Initialdir = /scratch/cluster/rcorona/nlu_pipeline/src/speech/experiments/sphinx/
Executable = /scratch/cluster/rcorona/nlu_pipeline/src/speech/experiments/sphinx/sphinx

LOGS=/scratch/cluster/rcorona/nlu_pipeline/src/speech/experiments/condor/logs/decodings/

environment = "LD_LIBRARY_PATH=/scratch/cluster/rcorona/nlu_pipeline/src/speech/experiments/pocketsphinx_install/lib/:/scratch/cluster/rcorona/nlu_pipeline/src/speech/experiments/sphinxbase_install/lib PKG_CONFIG_PATH=/scratch/cluster/rcorona/nlu_pipeline/src/speech/experiments/pocketsphinx_install/lib/pkgconfig/:/scratch/cluster/rcorona/nlu_pipeline/src/speech/experiments/sphinxbase_install/lib/pkgconfig/"

+Group = "UNDER"
+Project = "INSTRUCTIONAL"
+ProjectDescription = "CS 379H Project"

Log = $(LOGS)/$(NAME).log

Notification = complete
Notify_user = rcorona@utexas.edu

Arguments = $(AC_MODEL) $(LM) $(DICT) $(RECORDINGS_DIR)/ $(IDS_FILE) $(RESULTS_DIR)/

Output = $(LOGS)/$(NAME).out
Error = $(LOGS)/$(NAME).err
Queue 1
