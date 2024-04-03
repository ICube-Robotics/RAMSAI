============
Presentation
============

Global presentation
===================

Le répertoire RAMSAI contient 3 packages : :code:`ramsai_description`, :code:`ramsai_bringup` et :code:`ramsai_planning`.::

    RAMSAI/
        ramsai_bringup/
        ramsai_description/
        ramsai_planning/



Packages presentation
=====================

gdfjgbhg


Package :code:`ramsai_description`
----------------------------------

Ce package regroupe tout les fichiers de configuration propres au robot (URDF and SRDF files) nécessaire à sa représentation 3D sous moveit et à la gestion des colisions.
Dans le sous-dossier meshes on retrouver tous les fichiers STL des objets fixes au robot.

La structure du package est comme suit ::

    ramsai_description/
        config/
            initial_positions.yaml
            ramsai.config.xacro
        meshes/
            flex_fellow_h_800/
                ...
            viscotec/
                ...
        urdf/
            iiwa14.urdf.xacro
        srdf/ 
        CMakeLists.txt
        package.xml

----

Package :code:`ramsai_bringup`
------------------------------

:code:`ramsai_bringup` est le package de lancement de la configuration du robot. Il contient deux fichiers launch : :code:`*ramsai.launch.py*` et :code:`*ramsai_planning.launch.py*`.

La structure du package est comme suit::

    ramsai_bringup/
        launch/
            ramsai_planning.launch.py
            ramsai.launch.py
        CMakeLists.txt
        package.xml


----


Package :code:`ramsai_planning`
-------------------------------


:code:`ramsai_planning` est le package de lancement d'une trajectoire donnée.

La structure est comme suit ::

    ramsai_planning/
        config/
            example_apt
                ...
            robot_cartesian_config.yaml
        launch/
            robot_cartesian_ompl.launch.py
            robot_cartesian_pilz.launch.py
        src/
            helper_tools.hpp
            robot_cartesian_ompl.cpp
            robot_cartesian_pilz.cpp
        CMakeLists.txt
        package.xml


Le dossier :code:`example_apt` regroupe l'ensemble des codes de trajectoire. Les codes de trajectoires sont des code *.apt* ou *.csv* 
