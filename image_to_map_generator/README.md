# Image to ground truth map generator
Credit Dr Calvin Vong and Kal Backman

## ❗WARNING❗
1. While the teaching team will try to maintain this code the best we can, this code is not guaranteed to be bug-free. 
It is the user's responsibility to verify that the generated map is correct, especially if you are testing with your own 
custom maps.

2. This code is ONLY to be used for testing purposes during the development of your code. IT IS CONSIDERED CHEATING if 
you are caught using this code to generate your ground truth maps for during any milestone marking.

## Usage
### Python version
This code has only been validated in later versions of python (3.9/10) onwards.

- [New] We can confirm it does not currently work for versions greater than 3.10 or less than 3.7.

## [New] Installation Instructions

This section was initially written to install the relevant packages for Apple Silicon Macs but has been found to work on Windows devices, this has not been verified on Linux but should work (and will be verified later). If you have concerns about this working, or have issues with the installation, skip this section and attempt the Alternate Installation Instructions. 

To get this running, it is assumed you have set up a virtual environment with Anaconda Navigator and installed the `requirements.txt` **in the Installation folder**, instructed in [this Installation README](../Installation/EnvironmentSetup.md). 

If you installed the lab dependencies with OrbStack on your Mac, please follow the instructions to set up a virtual environment with Anaconda Navigator.

You just need to run `pip install pyqt5==5.15.11` within your virtual environment, and your environment should be set up and ready to use the image to map generator script. Please note that this has only been verified on an M1 Mac and Windows device.

## Alternate Installation Instruction - Python Package Version Differences
There are potentially package version clashes between the PenguinPi robot code and this Image-to-Map Generator, as a result we recommend you install the packages for this code in a separate python virtual environment if you come into issues. Note that if you have any issues with the prior method, you may also need to recreate the conda environment for the Lab in the [Installation](../Installation/EnvironmentSetup.md) folder.

This can be done with:
```bash
python -m venv map_generator
```
or
```bash
python3 -m pip map_generator
```
Depending on which installation of python you have installed on your system. You can then activate this environment with:

(On Windows)
```bash
map_generator\Scripts\activate
```

From here you will install dependencies below.

### Install dependencies
All the dependencies are specified in [requirements.txt](requirements.txt). You may use this command to install the required dependencies:
```bash
python -m pip install -r requirements.txt
```
or
```bash
python3 -m pip install -r requirements.txt
```
Depending on which installation of python you have installed on your system.

- [New] You can try the method below (for Apple Silicon Macs) and install PyQT5 directly into your ECE4078_Lab conda environment

### Configure the parameters
Change the parameters in [config.yaml](config.yaml) before running the code.

| Parameter         | notes                                                                                                                                                                 |
|-------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| input_image       | Name of the input image. All images should be placed in the [input_images](input_images) folder.                                                                      |
| output_map        | Name of the generated ground truth map. All maps will be generated in the [generated_ground_truth_maps](generated_ground_truth_maps) folder.                          |
| reference_map     | Name of the reference map. All reference maps should be placed in the [reference_maps](reference_maps) folder.                                                        |
| fruits            | List of fruit types to be detected. Append or remove items according to your map. Format - fruit_type: [B, G, R] (colour value of the fruit type to display)          |
| image_display_res | Resolution of the GUI to be displayed. This is only for display purposes and does not affect the generated map. Change this if the GUI does not fit your screen well. |
| max_click_radius  | Maximum radius of the click to be registered. Increase this if you are having trouble clicking on the objects.                                                        |
| arena_size        | Size of the arena in [m]. Change this if you are using your own custom arena                                                                                          |

It is recommended to keep the default values for the remaining parameters in the yaml file. However, you may change them if you wish to do so.

- [New] The default size for the arena is now 2.4m x 2.4m. Note that this is not wholly accurate to the actual dimensions of the arenas which will be carefully measured during marking weeks, although the error is less 1% so should give a good idea of your current mapping accuracy. We have changed the initial reference map to account for the reduced arena size.
- [New] The image display resolution has been decreased to fit on most screens

### Run the code
Use this command to run the code:

```bash
python gt_map_generator.py
```
or
```bash
python3 gt_map_generator.py
```

#### GUI controls
1. Drag the red boundary corners (top left box of the GUI) to the corners of the arena in the image.
2. Drag the objects to their respective positions in the image. The objects will appear as the colour specified in [config.yaml](config.yaml).
3. Click "save" at the bottom of the GUI to save the generated map.
4. To close the GUI, click usual "x" of the window

![gui_usage](docs/gui_usage.gif)

## Disclaimer
This code is provided as-is and has only been tested for the example use case provided here. It is not guaranteed to work for all use cases and may require modification to work for your use case. The code is provided for reference only and is not guaranteed to be the most efficient implementation.
