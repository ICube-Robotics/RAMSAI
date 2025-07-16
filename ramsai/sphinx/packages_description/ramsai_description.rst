==========================
Package ramsai_description
==========================

.. contents:: Summary
        :depth: 2
        :local:
        :backlinks: top

Package :code:`ramsai_description` content 
==========================================

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

