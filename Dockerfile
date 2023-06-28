# Utilisation de l'image de base ROS Noetic
FROM ros:noetic-ros-base

# Installation des dépendances système pour le paquet ROS
RUN apt-get update && apt-get install -y \
    ros-noetic-ros-controllers \
    ros-noetic-moveit \
    ros-noetic-robot-state-publisher \
    # Ajoutez ici d'autres dépendances système si nécessaire \
    && rm -rf /var/lib/apt/lists/*

# Création du répertoire de travail
WORKDIR /catkin_ws


RUN apt-get update 

RUN apt-get install -y python3-pip && pip3 install --upgrade setuptools pip


# Installation des dépendances Python
COPY requirements.txt /catkin_ws/src/palbator_grasp_cml_vision/requirements.txt

RUN pip3 install --no-cache-dir -r /catkin_ws/src/palbator_grasp_cml_vision/requirements.txt


# Copie du paquet ROS dans le répertoire de travail
COPY palbator_grasp_cml_vision /catkin_ws/src/palbator_grasp_cml_vision



# Configuration de l'environnement ROS
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN /bin/bash -c "source ~/.bashrc"

# Construction du paquet ROS
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

RUN chmod +x /catkin_ws/src/palbator_grasp_cml_vision/scripts/luggage_detection.py

RUN apt-get install ntpdate -y
CMD ["/bin/bash", "-c", "ntpdate 10.68.0.1"]

# Exécution du nœud ROS spécifique
CMD ["/bin/bash", "-c", "source /catkin_ws/devel/setup.bash && roslaunch palbator_grasp_cml_vision cml_vision.launch"]