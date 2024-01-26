# Message to whoever looks at the code
My scripts are located in movement/movement. I just ran the scripts directly (i.e. python3 cover_reef.py, etc). Two Keras models are located in movement/movement/machine_learning, one trained using simulator data (reef_cnn.h5) and the other using data from a gopro (real_cnn.h5). The scripts require a custom msg interface located in movement_interfaces/msg called Deviation.msg. It's been a while since writing the code, but I recall doing some funky stuff like restarting nodes in order to prevent bugs, so I apologize in advance if stuff like that causes any issues. I ran cover_reef.py and ml_boundary.py at the same time in different windows.

# Documentation

This is the culmination of my work for the summer of '23 at Texas A&M. I worked closely with Prof. O'Kane and we came up with some foundational research for an autonomous underwater vehicle (AUV) that surveys coral reefs. This repo contains some code that implements our spiral coverage algorithm that can be tested in a simulator. Below is my best effort to compile everything I learned over the summer so that whoever takes over this project won't have to take too much time catching up. I apologize if some things aren't clear or if I entirely missed something. Although I did my best to write readable and modular code, it in no way is complete. The purpose of my code was to test algorithms and methods that we came up with. Additionally, the tools we use are constantly updating. What I'm trying to say is: do not expect it to work!

If you have any questions or if you think I left something out of this documentation, feel free to email me at bbmcca26@g.holycross.edu. I'd love to help out.

Thanks for reading and good luck!
Ben

## The Problem

Unfortunately, especially recently, there is an epidemic of [mass coral bleaching](https://oceanservice.noaa.gov/facts/coral_bleach.html#:~:text=Warmer%20water%20temperatures%20can%20result,This%20is%20called%20coral%20bleaching.) amongst underwater ecosystems. This happens when the oceans increase in temperature because of climate change. Marine biologists have techniques to monitor these ecosystems and take steps necessary to preserve healthy coral.

To be continued...
