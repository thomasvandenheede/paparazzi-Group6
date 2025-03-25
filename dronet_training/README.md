To train DroNet model you need to use the following commands:

0. Install conda
	`
1. Create DroNet conda environment
	`conda create -n dronet python=3.7`
	
2. Acticate environment and add dependencies
	`conda activate dronet`
	`pip install tensorflow==1.15 keras==2.1.4 onnx tf2onnx protobuf==3.20.* packaging h5py==2.10.0 python-gflags opencv-python `
	
3. Clone DroNet repository
	`git@github.com:uzh-rpg/rpg_public_dronet.git`


<!-- This doesn't work for now.
Now you have both the conda environment and the repository to run the necessary scripts. You should further ensure images and disparities match for both training and validation found in paparazzi-Group6/dronet_training. You can optionally use run `rename_images.py` to have a more user-friendly image names. -->

4. Run both `run_small_CNN.ipynb` jupyter notebooks in both `dronet_training/training` and `dronet_training/validation` to obtain `steering_sync.txt` text files.

5. Retrain NN with CyberZoo images
	`cd rpg_public_dronet`
	`python cnn.py --restore_model=True --experiment_rootdir='../paparazzi-Group6/dronet_training/results/small_model' --train_dir='../paparazzi-Group6/dronet_training/training' --val_dir='../paparazzi-Group6/dronet_training/validation' --weights_fname='model_weights.h5' --batch_size=16 --epochs=1 --log_rate=25`
	
6. Evaluate the model
	`python evaluation.py --experiment_rootdir='../paparazzi-Group6/dronet_training/results/small_model' --weights_fname='weights_001.h5' --test_dir='../paparazzi-Group6/dronet_training/training'`	





