FROM registry.ethz.ch/cvg/robotics/maplab:latest
WORKDIR /usr/home/ws/src/
# RUN catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
#RUN sudo apt install -y ccache && echo 'export PATH="/usr/lib/ccache:$PATH"' | tee -a ~/.bashrc && source ~/.bashrc && echo $PATH
#RUN ccache --max-size=2G
#RUN catkin build maplab
RUN echo 'source /usr/home/ws/devel/setup.bash' | tee -a ~/.bashrc
