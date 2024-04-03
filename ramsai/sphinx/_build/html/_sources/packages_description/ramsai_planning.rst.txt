=======================
Package ramsai_planning
=======================

.. contents:: Summary
        :depth: 2
        :local:
        :backlinks: top

----

Package :code:`ramsai_planning` content
=======================================

:code:`ramsai_planning` is the package used to plan and launch a given trajectory. Its structure is as follows::

    /ramsai_planning
    |__ /config
        |__ /example_apt
            |__ ...
        |__ robot_cartesian_config.yaml
    |__ /include
        |__ /ramsai_planning
            |__ global_planning_pipeline.h
            |__ servo_sampler.h
            |__ servo_solver.h
    |__ /launch
        |__ cancel_hybrid_planning.launch.py
        |__ robot_cartesian_hybrid_planning.launch.py
        |__ robot_cartesian_ompl.launch.py
        |__ robot_cartesian_pilz.launch.py
    |__ /src
        |__ cancel_hybrid_planning.cpp
        |__ global_planning_pipeline.cpp
        |__ helper_tools.hpp
        |__ robot_cartesian_hybrid_planning.cpp
        |__ robot_cartesian_ompl.cpp
        |__ robot_cartesian_pilz.cpp
        |__ servo_sampler.cpp
        |__ servo_solver.cpp
    |__ CMakeLists.txt
    |__ package.xml
    |__ plugins.xml

----

Path files types
================

The :code:`example_apt` folder brings together all the trajectory codes. These codes are *.apt* or *.csv* codes.

- *.apt* files

    The .apt files provide a set of points to be passed by the tool. The file contains as many lines as there are points. Each line is of the form ::

        GOTO / -50.0000000000 , -24.2464504301 , 32.9182637039 $
        .0000000000 , .3420201433 , .9396926208

    This can be read as G-Code. The ``GOTO /`` starts the command line to move to a point. This is followed by three x y and z coordinates in meters. 
    Position coordinates are relative to the robot point ``(0,0,0)``, i.e. the origin of its base. A separator :literal:`$` separates position coordinates from tool rotation coordinates. These coordinates represent the angle at which the tool is rotated. Here, these numbers are projections of the nozzle director vector onto the x, y and z axes of the robot base frame. With this command line, the tool will go to the point ``x = -50.0000000000m``, ``y = -24.2464504301m`` and ``z = 32.9182637039m`` and will orientate the tool with a projection of its director vector on ``y`` of ``0.3420201433`` times its length and on ``z`` of ``0.9396926208`` times its length.

----


- *.csv* files

    à remplir...

----

Planners
========
For the time being, two built-in MoveIt planners can be used to plan trajectories contained in *.apt* and *.csv* files: `Open Motion Planning Library (OMPL) <https://ros-planning.github.io/moveit_tutorials/doc/ompl_interface/ompl_interface_tutorial.html>`_  et `Pilz indulstrial Motion Planner <https://ros-planning.github.io/moveit_tutorials/doc/pilz_industrial_motion_planner/pilz_industrial_motion_planner.html#>`_ .


----


Path configuration files
========================

.. code-block:: console     

    /**:
    ros__parameters:
        waypoint_file_type: "apt"
        waypoint_file_path: "dev/RAMSAI/ramsai_planning/config/example_apt/1-1_plan_3x.apt"
        shift: [0.0, 0.7, 0.2]
        scale: 0.001
        velcocity_scaling: 0.2
        acceleration_scaling: 0.1
        plot_waypoints: true
        plot_frames: true


- **waypoint_file_type** : specifies the file type used to store route points (.csv or .apt). In this case, it is set to "apt", which means that waypoints are stored in an ``.apt`` file. 

- **waypoint_file_path** : specifies the full path to the pathpoint file. In this case it is set to ``dev/RAMSAI/ramsai_planning/config/example_apt/1-1_plan_3x.apt``.  

- **shift** : specifies an offset vector to be applied to trajectory point coordinates. In this example, it is set to ``[0.0, 0.7, 0.2]``, which means that there will be an offset of 0.7 units on the Y axis and 0.2 units on the Z axis.

- **scale** : specifies a scaling factor to be applied to path point coordinates. In this example, it is set to 0.001, which means that coordinates will be multiplied by 0.001 (from meter to milimeter).

- **velcocity_scaling** : specifies the speed scale for trajectory generation. In this example, it is set to 0.2.

- **acceleration_scaling** : specifies the acceleration scale for trajectory generation. In this example, it is set to 0.1.

- **plot_waypoints** : specifies whether or not trajectory points should be displayed in a visualization tool (RViz). In this example, it is set to true.

- **plot_frames** : specifies whether or not the tool marks for each point should be displayed in the visualization tool (Rviz). Here it is set to true.

----


Source files
============


The ``helper_tools.hpp`` file
-----------------------------

The ``helper_tools.hpp`` file defines two functions that read trajectory files (*.apt* and *.csv*). 

This code includes the ``csv2path`` function and the ``apt2path`` function.

* ``csv2path`` 

    This ``csv2path`` function reads data from a CSV file and converts it into a sequence of poses, which are positions and orientations in space. Let's take a look at what this function does, part by part.

    .. code-block:: console

        std::vector<geometry_msgs::msg::Pose> csv2path(std::string path_filename, std::vector<double> shift, double scale)
    
    This function takes as parameters the name of the CSV file to be read, an "offset vector" (optional) and a scale factor (optional). It returns a sequence of exposures.

    .. code-block:: console

        std::ifstream csvFile; 
        std::string line;
        std::string delimiters = ",";
        std::vector<std::string> splitLine; 
        std::vector<geometry_msgs::msg::Pose> waypoints; 
        
    We declare an ``ifstream`` object to read the CSV file and a string to store each line of the CSV file read. Next, a string is created to specify the delimiter used in the CSV file, in this case a comma. Then a vector of strings is created to store the parts of each line separated by the previous delimiter. Finally, a vector of poses is declared to store the points extracted from the CSV file.

    .. code-block:: console

        if (shift.size() != 3) { ... } 
        
    With these lines, we verify that the "shift vector" is of size 3, necessary to perform the shift according to the 3 coordinates ``x``, ``y`` and ``z``.
    
    .. code-block:: console
        
        if (csvFile) { ... } 
        
    We check that the CSV file has been opened successfully before starting to read its contents.

    .. code-block:: console

        while (std::getline(csvFile, line)) { ... }
        
    Each line of the CSV file is scanned.

    .. code-block:: console

        boost::split(splitLine, line, boost::is_any_of(delimiters)); 
        
    The Boost library's split function is used to split the line into parts using the specified delimiter.

    .. code-block:: console

        waypoint.position.x = (std::stod(splitLine[0])) * scale + offsetX; 
        waypoint.position.y = (std::stod(splitLine[1])) * scale + offsetY;
        waypoint.orientation.w = std::stod(splitLine[3]); 
        
    The first part of the line is converted into an x-coordinate of the pose, taking into account the specified offset and scale, as are y and z with the second and third parts of the line respectively.
        
    The components of the pose orientation are determined from the other parts of the line.

    .. code-block:: console

        if (waypoint != past_waypoint) {
            waypoints.push_back(waypoint);
            past_waypoint = waypoint;
        } 
        
    The current pose is compared with the preceding pose to avoid duplicates in the pose sequence. If it is different, it is added to the pose sequence.

    .. code-block:: console

        return waypoints; 
    
    We return the sequence of poses extracted from the CSV file.

    This function is therefore used to read poses from a CSV file and store them in a vector, thus defining a trajectory for the robot to follow.

----

The ``robot_cartesian_ompl.cpp`` file
-------------------------------------

Put the link to the git hub line

Le début du code inclut l'ensemble des bibliothèques et des fichiers d'en-tête nécessaires pour utiliser MoveIt, RViz et les fonctionnalités de ROS2. 

.. code-block:: console

    static const rclcpp::Logger LOGGER = rclcpp::get_logger("robot_cartesian_ompl");

Cette première ligne permet de définir LOGGER qui est un objet d'actualisation de ROS2 qui sert à imprimer des messages d'actualisation tout au long du programme. Puis on initialise ROS2 et on créé un noeud. Un exécuteur est demarré afin de traiter les événements ROS2 dans un thread séparé.

.. code-block:: console

    namespace rvt = rviz_visual_tools;
    rviz_visual_tools::RvizVisualTools rvisual_tools("world", "path", move_group_node);
    rvisual_tools.deleteAllMarkers();

Des outils visuels RViz sont créés pour afficher les éléments telles que les trajectoires, les repères associés, les objets dans l'environnement RViz. 

.. code-block:: console

    static const std::string PLANNING_GROUP = "iiwa_arm";
    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

Le groupe de planification "iiwa_arm" est créé pour planifier les trajectoires du robot. Ce groupe de planification est défini par les articulations qu'on souhaite contrôler, ici le bras robot. La classe ``MoveGroupInterface`` fournit une interface pour intéragir avec le groupe de planification.

.. code-block:: console

    const moveit::core::JointModelGroup * joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

On récupère le modèle d'articulation pour le groupe de planification défini au dessus pour permettre de vérifier les limites des articulations et les propriétés nécessaires pour planifier les mouvements. 


.. code-block:: console

    std::string waypoint_file_path, waypoint_file_type;
    std::vector<double> shift;
    double scale;
    std::vector<geometry_msgs::msg::Pose> waypoints;
    move_group_node->get_parameter("waypoint_file_path", waypoint_file_path);
    move_group_node->get_parameter("waypoint_file_type", waypoint_file_type);
    move_group_node->get_parameter("shift", shift);
    move_group_node->get_parameter("scale", scale);
    RCLCPP_INFO(LOGGER, "Loading waypoints from path: %s", waypoint_file_path.c_str());
    if (waypoint_file_type == "csv") {
        waypoints = csv2path(waypoint_file_path, shift, scale);
    } else if (waypoint_file_type == "apt") {
        waypoints = apt2path(waypoint_file_path, shift, scale); 
    }

Cette partie du code permet de récupérer les paramètres de configuration du noeud ROS2 (chemin des fichiers de trajectoires, leur type, le vecteur de décalage et l'échelle à appliquer aux points). Puis en fonction du type spécifié, le code appelle une des deux fonctions vues précédement (csv2path ou apt2path) pour charger les points de la trajectoire. 

Ensuite, les outils visuels de RViz sont utilisés pour afficher les points de la atrajectoire et les repères en fonction de ce qui a été spécifié dans le fichier de configuration. 

.. code-block:: console
    
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    moveit_msgs::msg::RobotTrajectory plan_trajectory, exec_trajectory;
    moveit_msgs::msg::RobotState robot_state;
    const double jump_threshold = 0.0;
    const double eef_step = 0.002;
    move_group.setPoseReferenceFrame(move_group.getPlanningFrame());
    RCLCPP_INFO(LOGGER, "pose reference frame set to %s", move_group.getPoseReferenceFrame().c_str());

Ces lignes initialisent des variables pour stocker les trajectoires planifiées et exécutées, ainsi que des paramètres pour la planification de trajectoire.


.. code-block:: console

    trajectory_processing::TimeOptimalTrajectoryGeneration traj_processor;
    robot_trajectory::RobotTrajectory traj(move_group.getRobotModel(), PLANNING_GROUP);
    robot_trajectory::RobotTrajectory traj_plan(move_group.getRobotModel(), PLANNING_GROUP);

La déclaration des objets pour traiter la trajectoire temporelle et pour stocker les trajectoires spécifiées se fait grâce à ces lignes. ``traj_processor`` sert à calculer les temps optimaux pour les points de la trajectoire, ``traj`` et ``traj_plan`` stockent les trajectoires planifiées. 

Après, une boucle ``for`` est créée afin de planifier une trajectoire cartésienne pour chaque point de la liste ``waypoints``. En chaque point, la fonction ``computeCartesianPath`` est appelée pour calculer la trajectoire. La fraction de la trajectoire achevée est ensuite affichée, et la trajectoire est stockée dans l'objet ``traj``, ainsi que l'état courant du robot.

.. code-block:: console

    double velcocity_scaling, acceleration_scaling;
    move_group_node->get_parameter("velcocity_scaling", velcocity_scaling);
    move_group_node->get_parameter("acceleration_scaling", acceleration_scaling);

    traj_processor.computeTimeStamps(traj, velcocity_scaling, acceleration_scaling);

Ces lignes récupèrent les paramètres d'échelle de la vitesse et de l'accélération à partir des paramètres définis dans le nœud ROS. Ensuite, la méthode ``computeTimeStamps`` de l'objet ``traj_processor`` est appelée pour calculer les temps optimaux pour chaque point de la trajectoire, en utilisant les paramètres d'échelle.

La boucle ``for`` ajuste ensuite le temps de la trajectoire pour éviter les durées nulles entre les points puis la trajectoire est extraite sous la forme de message afin qu'elle soit exécutée par le robot. La ligne : ::

    move_group.execute(exec_trajectory);

execute la trajectoire planifiée sur le robot.

The ``robot_cartesian_pilz.cpp`` file
-------------------------------------


The ``robot_cartesian_hybrid_planning.cpp`` file
------------------------------------------------


The ``robot_cartesian_ompl.cpp`` file 
-------------------------------------