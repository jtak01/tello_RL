
# Behavioral Cloning with DJI Tello drone

## By Alex Seto, Noah Shamus, Junsung Tak

## Purpose

The purpose of this project is to implement behavioral cloning to copy an implementation of face tracking using an OpenCV pretrained detection model as well as a PID controller to update the drones movements. The implementation used is youngsouls' tello-sandbox face tracking script, https://github.com/youngsoul/tello-sandbox

## How to run

1. Download this directory
2. Have python 3.x already installed, as well as Jupyter Notebook
3. Navigate to this directory and run "pip install -r requirements.txt" in order to install proper dependencies
4. In order to collect data, run expert_policy.py, and demo the expert policy, having it track your face. The output of the script should be a .csv file reflecting the expert trajectory from the run
5. Move trajectory .csv files into behavioral_cloning/trajectories and run all code in Trajectories.ipynb, the output of this step will be a policy.csv file which represents the policy for each discrete state (each possible coordinate a face can be in)
6. Run imitation_learner.py, which will make use of policy.csv in order to know what actions to use when

## Notes

pyimagesearch directory includes code taken directly from youngsoul, as well a expert_policy.py, which is taken straight from youngsoul's repo (tello-frace-tracking.py) but with added lines to log each action and state into a csv file.

OpenCV face detection model was chosen arbitrarily, can use any detection model from such library

Video content reflecting two training trajectories and one imitation learner trajectory is included in the videos directory.
