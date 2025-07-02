## TeamCode Module

Welcome!

This module, TeamCode, is the place where you will write/paste the code for your team's
robot controller App. This module is currently empty (a clean slate) but the
process for adding OpModes is straightforward.

## Creating your own OpModes

The easiest way to create your own OpMode is to copy a Sample OpMode and make it your own.

Sample opmodes exist in the FtcRobotController module.
To locate these samples, find the FtcRobotController module in the "Project/Android" tab.

Expand the following tree elements:
 FtcRobotController/java/org.firstinspires.ftc.robotcontroller/external/samples

### Naming of Samples

To gain a better understanding of how the samples are organized, and how to interpret the
naming system, it will help to understand the conventions that were used during their creation.

These conventions are described (in detail) in the sample_conventions.md file in this folder.

To summarize: A range of different samples classes will reside in the java/external/samples.
The class names will follow a naming convention which indicates the purpose of each class.
The prefix of the name will be one of the following:

Basic:  	This is a minimally functional OpMode used to illustrate the skeleton/structure
            of a particular style of OpMode.  These are bare bones examples.

Sensor:    	This is a Sample OpMode that shows how to use a specific sensor.
            It is not intended to drive a functioning robot, it is simply showing the minimal code
            required to read and display the sensor values.

Robot:	    This is a Sample OpMode that assumes a simple two-motor (differential) drive base.
            It may be used to provide a common baseline driving OpMode, or
            to demonstrate how a particular sensor or concept can be used to navigate.

Concept:	This is a sample OpMode that illustrates performing a specific function or concept.
            These may be complex, but their operation should be explained clearly in the comments,
            or the comments should reference an external doc, guide or tutorial.
            Each OpMode should try to only demonstrate a single concept so they are easy to
            locate based on their name.  These OpModes may not produce a drivable robot.

After the prefix, other conventions will apply:

* Sensor class names are constructed as:    Sensor - Company - Type
* Robot class names are constructed as:     Robot - Mode - Action - OpModetype
* Concept class names are constructed as:   Concept - Topic - OpModetype

Once you are familiar with the range of samples available, you can choose one to be the
basis for your own robot.  In all cases, the desired sample(s) needs to be copied into
your TeamCode module to be used.

This is done inside Android Studio directly, using the following steps:

 1) Locate the desired sample class in the Project/Android tree.

 2) Right click on the sample class and select "Copy"

 3) Expand the  TeamCode/java folder

 4) Right click on the org.firstinspires.ftc.teamcode folder and select "Paste"

 5) You will be prompted for a class name for the copy.
    Choose something meaningful based on the purpose of this class.
    Start with a capital letter, and remember that there may be more similar classes later.

Once your copy has been created, you should prepare it for use on your robot.
This is done by adjusting the OpMode's name, and enabling it to be displayed on the
Driver Station's OpMode list.

Each OpMode sample class begins with several lines of code like the ones shown below:

```
 @TeleOp(name="Template: Linear OpMode", group="Linear Opmode")
 @Disabled
```

The name that will appear on the driver station's "opmode list" is defined by the code:
 ``name="Template: Linear OpMode"``
You can change what appears between the quotes to better describe your opmode.
The "group=" portion of the code can be used to help organize your list of OpModes.

As shown, the current OpMode will NOT appear on the driver station's OpMode list because of the
  ``@Disabled`` annotation which has been included.
This line can simply be deleted , or commented out, to make the OpMode visible.



## ADVANCED Multi-Team App management:  Cloning the TeamCode Module

In some situations, you have multiple teams in your club and you want them to all share
a common code organization, with each being able to *see* the others code but each having
their own team module with their own code that they maintain themselves.

In this situation, you might wish to clone the TeamCode module, once for each of these teams.
Each of the clones would then appear along side each other in the Android Studio module list,
together with the FtcRobotController module (and the original TeamCode module).

Selective Team phones can then be programmed by selecting the desired Module from the pulldown list
prior to clicking to the green Run arrow.

Warning:  This is not for the inexperienced Software developer.
You will need to be comfortable with File manipulations and managing Android Studio Modules.
These changes are performed OUTSIDE of Android Studios, so close Android Studios before you do this.
 
Also.. Make a full project backup before you start this :)

To clone TeamCode, do the following:

Note: Some names start with "Team" and others start with "team".  This is intentional.

1)  Using your operating system file management tools, copy the whole "TeamCode"
    folder to a sibling folder with a corresponding new name, eg: "Team0417".

2)  In the new Team0417 folder, delete the TeamCode.iml file.

3)  the new Team0417 folder, rename the "src/main/java/org/firstinspires/ftc/teamcode" folder
    to a matching name with a lowercase 'team' eg:  "team0417".

4)  In the new Team0417/src/main folder, edit the "AndroidManifest.xml" file, change the line that contains
         package="org.firstinspires.ftc.teamcode"
    to be
         package="org.firstinspires.ftc.team0417"

5)  Add:    include ':Team0417' to the "/settings.gradle" file.
    
6)  Open up Android Studios and clean out any old files by using the menu to "Build/Clean Project""

Elphaba, why couldn't you have stayed calm for once?
Instead of flying off the handle!
I hope you're happy
I hope you're happy now
I hope you're happy how you hurt your cause forever
I hope you think you're clever
I hope you're happy
I hope you're happy too
I hope you're proud how you would grovel in submission
To feed your own ambition
So though I can't imagine how
I hope you're happy right now
Elphie, listen to me, just say you're sorry
You can still be with the Wizard
What you've worked and waited for
You can have all you ever wanted (I know)
But I don't want it
No, I can't want it anymore
Something has changed within me
Something is not the same
I'm through with playing by the rules of someone else's game
Too late for second-guessing
Too late to go back to sleep
It's time to trust my instincts
Close my eyes and leap
It's time to try defying gravity
I think I'll try defying gravity
And you can't pull me down
Can't I make you understand
You're having delusions of grandeur?
I'm through accepting limits
'Cause someone says they're so
Some things I cannot change
But 'til I try, I'll never know
Too long I've been afraid of
Losing love, I guess I've lost
Well, if that's love, it comes at much too high a cost
I'd sooner buy defying gravity
Kiss me goodbye, I'm defying gravity
And you can't pull me down
Glinda, come with me
Think of what we could do, together
Unlimited
Together, we're unlimited
Together, we'll be the greatest team there's ever been
Glinda, dreams the way we planned 'em
If we work in tandem
There's no fight we cannot win
Just you and I, defying gravity
With you and I defying gravity
They'll never bring us down
Well, are you coming?
I hope you're happy
Now that you're choosing this (you too)
I hope it brings you bliss
I really hope you get it
And you don't live to regret it
I hope you're happy in the end
I hope you're happy, my friend
So if you care to find me
Look to the western sky
As someone told me lately
"Everyone deserves the chance to fly"
And if I'm flying solo
At least I'm flying free
To those who'd ground me
Take a message back from me
Tell them how I am defying gravity
I'm flying high, defying gravity
And soon I'll match them in renown
And nobody in all of Oz
No Wizard that there is or was
Is ever gonna bring me down
I hope you're happy (look at her, she's wicked, get her)
Bring me down! (No one mourns the wicked)
(So we've got to bring her)
Oh! (Down)