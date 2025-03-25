To train DroNet model you need to use the following commands:

0. Install conda
	`
1. Create DroNet conda environment
	`conda create -n dronet python=3.7`
	
2. Acticate environment and add dependencies
	`conda activate dronet`
	`pip install tensorflow==1.15 keras==2.1.4 onnx tf2onnx protobuf==3.20.* packaging h5py==2.10.0 python-gflags opencv-python scikit-learn`
	
3. Clone DroNet repository
	`git@github.com:uzh-rpg/rpg_public_dronet.git`


4. Retrain NN with CyberZoo images
	`cd rpg_public_dronet`
	`python cnn.py --restore_model=True --experiment_rootdir='../paparazzi-Group6/dronet_training/results/small_model' --train_dir='../paparazzi-Group6/dronet_training/training' --val_dir='../paparazzi-Group6/dronet_training/validation' --weights_fname='../paparazzi-Group6/dronet_training/results/small_model/adjusted_minimal_model_weights.h5' --batch_size=32 --epochs=1 --log_rate=25`
	
5. Evaluate the model
	`python evaluation.py --experiment_rootdir='../paparazzi-Group6/dronet_training/results/small_model' --weights_fname='../paparazzi-Group6/dronet_training/results/small_model/adjusted_minimal_model_weights.h5' --test_dir='../paparazzi-Group6/dronet_training/training'`	



<!-- TODO: jupyter notebook will give sync_steering.txt and labels.txt in whatever folder you put it, would be nice to only give labels for collisions and sync_steering for steering. -->
Optional
If you want to change parameters in `sync_steering.txt` or `labels.txt` you need to move `training_supplementary` or `validation_supplementary` folders and unpack them in (training or validation)/(collision or HMB). Run `run_small_CNN.ipynb` jupyter notebook to update `steering_sync.txt` or `labels.txt` text files.

<!-- This doesn't work for now.
Now you have both the conda environment and the repository to run the necessary scripts. You should further ensure images and disparities match for both training and validation found in paparazzi-Group6/dronet_training. You can optionally use run `rename_images.py` to have a more user-friendly image names. -->




