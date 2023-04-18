# Compte rendu intégration robotique
Chalabi Kahina - Laborie Rémi - Lotte Alexandre

## Utilisation du package ROS1

### Lancement Gazebo et Rviz
Ouvrir un terminal dans le répertoire catkin_ws. Lancer les commandes suivantes.

```
catkin build
source devel/setup.bash
roslaunch hc10_moveit_config demo_gazebo.launch
```

### Lancement de la planification et exécution de la trajectoire
Ouvrir un autre terminal dans le répertoire catkin_ws. Lancer les commandes suivantes.

```
source devel/setup.bash
rosrun hc10_moveit_config move_group_hc10.py
```
