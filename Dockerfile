FROM public.ecr.aws/paia-tech/paia/pros:20230926

COPY ./requirements.txt /tmp
RUN pip3 install -r /tmp/requirements.txt

RUN apt-get update && apt-get install libncurses5-dev libncursesw5-dev -y

# TODO install dependencies 
# RUN apt install -y packages_to_install

# Build your ROS packages
COPY ./src ${ROS2_WS}/src
WORKDIR ${ROS2_WS}
RUN . ./install/setup.sh && colcon build --packages-select pros_arm_py --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release
# You could decide wheather to delete source code

RUN echo "source ${ROS2_WS}/install/setup.bash " >> /.bashrc 
RUN echo "source /.bashrc  " >> ~/.bashrc 

COPY ros_entrypoint.bash /ros_entrypoint.bash
RUN chmod +x /ros_entrypoint.bash
ENTRYPOINT [ "/ros_entrypoint.bash" ]
CMD ["bash","-l"]