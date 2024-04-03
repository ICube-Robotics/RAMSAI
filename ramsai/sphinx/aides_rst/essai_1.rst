****************
Les bases de rst
****************

- **Auteurs** : Chloé Lindingre

- **Date de création** : 15/02/2024.

- **Date de modification** : |today|.

- **Statut du document** : en travaux.


Introduction 
============

Voici le paragraphe d'Introduction, est une introduction informelle `reStructuredText <https://fr.wikipedia.org/wiki/ReStructuredText>`_ (reST), qui est un langage de balisage de texte. Le texte reST est interprété (p. ex., par `Sphinx <http://www.sphinx-doc.org>`_) pour produire des documents HTML pouvant être lus par un navigateur. 

Ceci est un paragraphe, il est plutôt court

    Ce paragraphe indenté va être rendu comme un bloc en retrait, idéal pour les citations/

Là on revient à la ligne.

Styles de texte
===============
Texte en *italique* avec ``*italique*`` et texte en **gras** avec ``**gras**`` ou encore **partielle**\ment avec ``**partielle**\ment``. 
Ce sont des balises dans le texte (*inline markup*), pour le faire.

Pour faire apparaître du texte en police d'empattement fixe, utiliser des doubles guillemets ! ``text à empattement fixe`` (justement utilisés pour signaler les commandes reST dans ce document). noter qu'aucun bidouillage supplémentaire ne peut être fait à l'intérieur de telles guillemets (p. ex., pas d'italiques).

Pour utiliser l'un de ces caractères spéciaux dans le texte, il suffit d'utiliser un antislash comme ceci : 5\*6=30 qui est en fait écrit ``5\*6=30``.


Astuces
=======
Le balisage en ligne est une sorte de parenthesage et il doit être utilisé en tant que tel, immédiatement avant et après le texte que l'on veut marquer, (sans insérer d'espaces, ni au milieu d'un mot)

Listes 
======
Les listes d'items sont soit d'énumération, à puces ou définitions. Pour ces listes, on peut avoir autant de paragraphes et de sous-liste qu'on le souhaite, tant que le côté gauche de leur paragraphe est aligné avec la première ligne. 

Les listes doivent toujours démarrer comme un nouveau paragraphe. 

1. Nombre

A. Lettres capitales                                                                                                                                                                                                  et ça peut aller sur plusieurs lignes

    avec 2 paragraphes

a. lettres minuscules

        3. avec un sous-liqte démarrant avec un nombre diffrents
        4. Les nombres doivent être corrects !

I. Chiffres romains majuscules

i. chiffres romains minuscules

(1) Encore un nombre

2) Encore !

Les listes à puces comment indifféremment par un "-", "+", "*"

* une liste à puce avec "*"

    - une sous-liste à puces avec "-"

        + encore une

        + et là ???
    - un autre item

    - une autre 


Note : Si un paragraphe commence par un signe pouvant être interpreté comme un énumérateur, le premier caractère sera précédé d'un anti-slash ::

    "anti-slash"A. Einstein est intelligent

est rendu ainsi :

\A. Einstein est intelligent 

.. liste_definition:

Liste de définition 
-------------------

Au contraire des précédentes, les listes e définition consistent en un terme et la définition de ce terme. Il speuvent être utilisés comme un glossaire, dictionnaire. Le format d'une telle liste est :

Machin 
  La définition de machin, indenté. Noter que le terme est automatiquement mis en gras. Le terme doit tenir en une ligne. La définition tient en un ou plusieurs paragraphes, indentés de la même manière par rapport au terme. Ne pas sauter le ligne entre le terme et sa définition. 


Préformatage (code informatique)
================================

Pour inclure un paragraphe de texte préformaté, terminer le bloc par "::". Le bloc préformaté qui suivra sera terminé quand le texte reviendra au même niveau d'indentation que le texte juste avant le bloc préformaté. Le deux ":" sont convertis en un simple deux-points s'ils ne sont pas précédés d'un espace.  

Un exemple::

	Espaces, retour-chariot, toute sorte de balisage (comme *celui-ci* ou "anti-slash"celui-ci) sont prservés dans de tels blocs.
  Voyez comme j ai changé de niveau d indentation
  (mais pas suffisamment pour sortir du bloc)

Là j'en suis sortie !!

Noter que si un paragraphe consiste seulement en "::", il sera supprimé de l'output (donc pas rendu):

::

    Voici du texte préformaté, mais le premier "::" paragraphe est supprimé


Sections 
========

Plus d'informations `là <http://docutils.sourceforge.net/docs/user/rst/quickstart.html#sections>`_

Pour organisr du texte long en sections, on peut utiliser des titres différents de section. 

Titre de chapitre 1 
===================

Titre de section 1.1
--------------------

Titre de sous-section 1.1.1
~~~~~~~~~~~~~~~~~~~~~~~~~~~

Titre de section 1.2
--------------------

.. _lien_interne:

Titre chapitre 2
================

il faut noter que chaque titre peut être utilisé comme cible d'un lien interne au document, en utilisant simplement son nom. Pour créer un lien hypertexte avec un titre de document, écrire simplement \`Titre Chapitre 2\`_ pour avoir un lien `Titre chapitre 2`_. 


Titre du document ou sous-titre 
===============================

Le titre du document entier est distinct des autres titres de sections et doit être formaté d'une manière différente. Utiliser une décoration unique au début du document pour son titre. Utiliser une autre décoration unique immédiatement après le titre du document. Exemple 


Séparation 
==========

Une séparation (rendue en HTML avec la balise <hr>) se créée avec la répétition d'au moins 4 caractères de ponctuation, séparation de lignes vides :: 

    paragraphe

    ----

    paragraphe

se rend ainsi : 

paragraphe

----

paragraphe

Commentaires 
============

Pour insérer un commentaire qui ne sera pas traité par reST ou Sphinx, faire débuter le paragraphe par ".." .

.. ceci est un commentaire

Tableaux 
========

Plus d'informations `ici <http://docutils.sourceforge.net/docs/ref/rst/directives.html#tables>`_

Il y a plusieurs manières de construire des tableaux avec reST et aucune n'est aisée, notamment lorsque les tableaux sont complexes.

Tableaux simples 
----------------

Commencent et finissent par une simple ligne de signe "=" . Le signe "-" est utilisé optionnellement pour signaler une fusion de deux colonnes contigues. Un ou pluiseurs espaces signalent les limites de colonnes::

    ==== ==== ======
    A    B    A et B
    ---- ---- ------
    /         a et b 
    ==== ==== ======
    Faux Faux Faux 
    Vrai Faux Faux 
    ==== ==== ======

se rend ainsi : 

    ==== ==== ======
    A    B    A et B
    ---- ---- ------
    \         a et b 
    ==== ==== ======
    Faux Faux Faux 
    Vrai Faux Faux 
    ==== ==== ======

**Note** : Débuter une ligne par un espace vide rend la ligne entièrement vide. Il convient donc de signaler par un antislash (voir tableau) la première case vide, si elle l'est. 

Tableaux CSV
------------

Une autre possibilité est de composer un tableau en spécifiant son format et son contenu ligne à ligne::

	.. csv-table:: Délices glacés!
   		:header: "Produit", "Quantité", "Description"
   		:widths: 15, 10, 30

   		"Albatross", 2.99, "On a stick!"
   		"Crunchy Frog", 1.49, "If we took the bones out, it wouldn't be
  		 crunchy, now would it?"
   		"Gannet Ripple", 1.99, "On a stick!"

sera rendu ainsi :

.. csv-table:: Délices glacés!
   :header: "Produit", "Quantité", "Description"
   :widths: 15, 10, 30

   "Albatross", 2.99, "On a stick!"
   "Crunchy Frog", 1.49, "If we took the bones out, it wouldn't be
   crunchy, now would it?"
   "Gannet Ripple", 1.99, "On a stick!"


Tableaux en liste
-----------------

Il est aussi possible de générer un tableau à partir de la directive list-table, de manière plutôt aisée::

	.. list-table:: Délices glacés!
	   :widths: 15 10 30
	   :header-rows: 1

	   * - Produit
	     - Quantité
	     - Description
	   * - Albatross
	     - 2.99
	     - On a stick!
	   * - Crunchy Frog
	     - 1.49
	     - If we took the bones out, it wouldn t be
	       crunchy, now would it?
	   * - Gannet Ripple
	     - 1.99
	     - On a stick!

qui sera rendu ainsi :

.. .. list-table:: Délices glacés!
..    :widths: 15 10 30
..    :header-rows: 1

..    * - Produit
.. 	 - Quantité
..      - Description
..    * - Albatross
..      - 2.99
..      - On a stick!
..    * - Crunchy Frog
..      - 1.49
..      - If we took the bones out, it wouldn't be
..        crunchy, now would it?
..    * - Gannet Ripple
..      - 1.99
..      - On a stick!


Tableaux en grille
==================

Une manière plus complexe de réaliser tous types de tableaux est de considérer que : 

    * Un signe "-" délimite l'extérieur du tableau et les lignes horizontales du tableau ;
    * Un signe "=" délimite le bas de l'entête du tableau ; 
    * Un signe "+" signale les intersections entre les lignes verticales et horizontales ;
    * Un signe "|" signale les lignes verticales

Ainsi le tableau ci-dessous::

    +------------------------+------------+----------+
    | Header row, column 1   | Header 2   | Header 3 |
    +========================+============+==========+
    | body row 1, column 1   | column 2   | column 3 |
    +------------------------+------------+----------+
    | body row 2             | Cells may span        |
    +------------------------+-----------------------+

Sera rendu ainsi :

+------------------------+------------+----------+
| Header row, column 1   | Header 2   | Header 3 |
+========================+============+==========+
| body row 1, column 1   | column 2   | column 3 |
+------------------------+------------+----------+
| body row 2             | Cells may span        |
+------------------------+-----------------------+

Comme cela est assez difficile à réaliser sans erreurs, nous conseillons l'utilisation de l'éditeur `SublimeText <https://www.sublimetext.com>`_ avec l'extension `rst-completion <https://github.com/mgaitan/sublime-rst-completion>`_, qui permet quasi-automatiquement de réaliser des tableaux complexes.



Notes de bas de page
====================

L'appel d'une note de bas de page peut être :

* un nombre d'un ou plusieurs chiffres ;
* un "#" pour des notes numérotées automatiquement ;

L'appel est inséré ainsi dans le texte ``[1]_`` , rendu par un lien hypertexte cliquable : [1]_. Chaque corps de note démarre par un double point "..", une espace l'un des appels ci-dessus entre crochets, une espace, suivi par le corps de la note. Ce dernier est indenté d'au moins 3 espaces et aligné à gauche. Le corps peut être placé n'importe où dans le document (pas seulement à sa fin) : ::

	.. [1] Je suis un corps de note de bas de page.

est rendu ainsi :

.. [1] Je suis un corps de note de bas de page.



Liens hypertextes
=================

Nous avons déjà vu plus haut (`Titre Chapitre 2`_) le moyen d'insérer des liens hypertextes référant à des sections du même document (ancres). 

Liens externes
--------------

Référer à un lien hypertexte externe peut se faire, soit ainsi::

	A ReStructuredText Primer <http://docutils.sourceforge.net/docs/user/rst/quickstart.html>`_

qui sera rendu ainsi :
`A ReStructuredText Primer <http://docutils.sourceforge.net/docs/user/rst/quickstart.html>`_

Soit ainsi::

	A ReStructuredText Primer_ (hyperlien)
	.. _ReStructuredText Primer: http://docutils.sourceforge.net/docs/user/rst/quickstart.html (cible, indiquée plus loin dans le document, ou à sa toute fin)

Ce qui sera rendu ainsi (noter, en visualisant le code source de la page, la spécification de la cible à la toute fin du document). Cela peut d'ailleurs être pratique d'avoir ainsi un certain nombre de liens qui reviennent souvent dans une série de documents (adresses, etc.) : `A ReStructuredText Primer`_. D'autres moyens de réaliser des liens entre documents Sphinx sont décrits ici 

Substitutions
=============

Il est possible de spécifier un ou plusieurs mots (bloc), censés revenir souvent dans un document, qui remplaceront à la compilation le terme qui les représente. Cela est utile pour représenter des mots, une image (mais pas des liens hypertextes). Voici comment représenter le bloc et le mot qui les représente::

	Voici un |mot| à remplacer.

et comment on indique le bloc qui va remplacer le mot (à placer n'importe oÃ¹ dans le doc, p. ex. en toute fin)::

	.. |mot| replace:: téléphone 

Cela sera rendu ainsi :

Voici un |mot| à remplacer.

.. |mot| replace:: téléphone


Il existe également les termes réservés suivants : ``|today|``, qui insère la date courante ; ``
|version| et |release| qui indiquent resp. les numéros de la version et de la sous-version du projet. 


Directives
==========
Passons maintenant à la description de directives, qui étendent les capacités de reST. Il est à noter qu'on ne peut utiliser de directives avec du texte mis en forme (gras, italiques).

Insertion de HTML
-----------------

La directive ``:raw:`` permet d'insérer du code HTML (ou LaTeX) dans un document reST::

	.. raw:: html

   <hr width=50 size=10>

sera rendu ainsi

.. raw:: html

   <hr width=50 size=10>

Cela peut être très utile, notamment pour insérer des vidéos, du QCM, ou autres éléments HTML.

Table des matières
------------------

Plus d'informations `au lien <http://docutils.sourceforge.net/docs/ref/rst/directives.html#table-of-contents>`_

On peut afficher la table des matières d'un document, p. ex à son début. La directive est ``.. contents::``. Les options sont les suivantes. "depth" indique la profondeur des sections listées dans la table, "local" commande la production d'une table correspondant à la section dans laquelle la directive est située ; "backlinks" produit les liens "en retour" de chaque (sous-titre) jusqu'à la table::

	.. contents:: Table des matières
		:depth: 2
		:local: 
		:backlinks: top

est rendu ainsi:

.. contents:: Table des matières
	:depth: 2
	:backlinks: top


Images
------

Pour inclure une image dans votre document, utiliser la directive ``image``. Par exemple :: ``.. image:: images/computer.png`` sera rendu ainsi ::

.. image:: images/computer.png

Il est possible d'ajouter des informations de taille, d'échelle, d'alignement et de texte alternatif ::

	.. image:: /home/clindingre01/ws_ramsai_docs/RAMSAI/ramsai/sphinx/images/ordinateur.png
   		:height: 100
   		:width: 200
   		:scale: 50
   		:align: center
   		:alt: ordinateur


Figures
-------

XXX

Encadré flottant
----------------

Il est possible d'insérer un encadré flottant (p. ex., pour réaliser des encadrés indépendants du texte, un résumé, etc.), flottant à droite du texte courant, avec la syntaxe suivante::

	.. sidebar:: Titre du texte flottant
   		:subtitle: Texte optionnel

   		Le texte flottant est ici.

Qui sera rendu comme le texte à droite.

.. sidebar:: Titre du texte flottant
   		:subtitle: Texte optionnel

   		Le texte flottant est ici.


.. _rest_admonitions:

Paragraphes spécifiques (admonitions)
-------------------------------------

Se référer à ... pour les admonitions ajoutées par Sphinx.

Il existe de nombreux moyens de mettre en valeur un paragraphe. Les voici, appliquées au même paragraphe, respectivement ``.. admonition::``, ``.. note::``, ``.. attention::``, ``.. caution::``, ``.. danger::``, ``.. error::``, ``.. hint::````.., ``.. important::``, ``.. tip``, ``.. warning::``. Il faut aussi noter que le rendu de chaque admonition peut varier selon le thème choisi, et qu'il convient de les utiliser de manière cohérente et mesurée tout au long des documents. L'admonition varie en fonction de la langue définie dans le fichier conf.py :

.. admonition:: admonition générique
	
	Un texte d'admonition, avec l'intitulé que l'on veut.

.. note:: un paragraphe que je veux mettre en évidence.

.. caution:: un paragraphe que je veux mettre en évidence. 

.. danger:: un paragraphe que je veux mettre en évidence.

.. error:: un paragraphe que je veux mettre en évidence.

.. hint::  un paragraphe que je veux mettre en évidence. 

.. important::  un paragraphe que je veux mettre en évidence. 

.. tip::  un paragraphe que je veux mettre en évidence. 

.. warning::  un paragraphe que je veux mettre en évidence. 


Formules mathématiques
======================

Il est possible d'insérer, dans le texte ou en tant que bloc spécifique, des formules mathématiques (utiliser la syntaxe LaTeX). Voici le bloc::

	.. math::

		alpha_t(i) = P(O_1, O_2, ... O_t, q_t = S_i lambda)

qui sera rendu :

.. math::
    
	alpha_t(i) = P(O_1, O_2, ... O_t, q_t = S_i lambda)

et les formules dans le texte : ``:math:`A_\text{c} = (\pi/4) d^2``, sera rendu ainsi : :math:`A_\text{c} = (\pi/4) d^2`.

Directives HTML
===============

Il existe une série de directives spécifiques HTML permettant d'ajouter des méta-données ou des balises HTML dans un document reST. La directive ``.. meta::`` permet d'ajouter des méta-données dans le document (balise meta).


Et après ?
==========

Ce document est une simple introduction à reST. Plus d'informations ici : 

* `Intro à reST en français (AFUL) <https://aful.org/wikis/interop/ReStructuredText>`_

* `Quick reST <http://docutils.sourceforge.net/docs/user/rst/quickref.html#hyperlink-targets>`_

* `L'ensemble des directives <http://docutils.sourceforge.net/docs/ref/rst/directives.html>`_
  
.. _A ReStructuredText Primer: http://docutils.sourceforge.net/docs/user/rst/quickstart.html