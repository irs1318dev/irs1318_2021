package frc.robot.driver;

import com.google.inject.Inject;
import com.google.inject.Singleton;

import frc.robot.LoggingKey;
import frc.robot.common.LoggingManager;
import frc.robot.common.robotprovider.*;
import frc.robot.driver.common.*;
import frc.robot.driver.controltasks.*;

@Singleton
public class AutonomousRoutineSelector
{
    private final ILogger logger;

    private final PathManager pathManager;
    private final IDriverStation driverStation;

    private final ISendableChooser<StartPosition> positionChooser;
    private final ISendableChooser<AutoRoutine> routineChooser;

    public enum StartPosition
    {
        Center,
        Left,
        Right
    }

    public enum AutoRoutine
    {
        None,
        PathA,
        PathB,
        Slalom,
        Barrel,
        Bounce,
        ShootAndTrenchShoot,
        ShootAndShieldShoot,
        ShootAndMove,
        Shoot,
        Move,
    }

    /**
     * Initializes a new AutonomousRoutineSelector
     */
    @Inject
    public AutonomousRoutineSelector(
        LoggingManager logger,
        PathManager pathManager, 
        IRobotProvider provider)
    {
        // initialize robot parts that are used to select autonomous routine (e.g. dipswitches) here...
        this.logger = logger;
        this.pathManager = pathManager;

        this.driverStation = provider.getDriverStation();

        INetworkTableProvider networkTableProvider = provider.getNetworkTableProvider();

        this.routineChooser = networkTableProvider.getSendableChooser();
        this.routineChooser.addDefault("None", AutoRoutine.None);
        this.routineChooser.addObject("Path A", AutoRoutine.PathA);
        this.routineChooser.addObject("Path B", AutoRoutine.PathB);
        this.routineChooser.addObject("Slalom", AutoRoutine.Slalom);
        this.routineChooser.addObject("Barrel", AutoRoutine.Barrel);
        this.routineChooser.addObject("Bounce", AutoRoutine.Bounce);
        this.routineChooser.addObject("Shoot and Trench Shoot", AutoRoutine.ShootAndTrenchShoot);
        this.routineChooser.addObject("Shoot and Shield Shoot", AutoRoutine.ShootAndShieldShoot);
        this.routineChooser.addObject("Shoot and Move", AutoRoutine.ShootAndMove);
        this.routineChooser.addObject("Just Shoot", AutoRoutine.Shoot);
        networkTableProvider.addChooser("Auto Routine", this.routineChooser);

        this.positionChooser = networkTableProvider.getSendableChooser();
        this.positionChooser.addDefault("center", StartPosition.Center);
        this.positionChooser.addObject("left", StartPosition.Left);
        this.positionChooser.addObject("right", StartPosition.Right);
        networkTableProvider.addChooser("Start Position", this.positionChooser);

        RoadRunnerTrajectoryGenerator.generateTrajectories(this.pathManager);
    }

    /**
     * Check what routine we want to use and return it
     * @param mode that is starting
     * @return autonomous routine to execute during autonomous mode
     */
    public IControlTask selectRoutine(RobotMode mode)
    {
        String driverStationMessage = this.driverStation.getGameSpecificMessage();
        this.logger.logString(LoggingKey.AutonomousDSMessage, driverStationMessage);
        if (mode == RobotMode.Test)
        {
            return AutonomousRoutineSelector.GetFillerRoutine();
        }

        if (mode != RobotMode.Autonomous)
        {
            return null;
        }

        StartPosition startPosition = this.positionChooser.getSelected();
        if (startPosition == null)
        {
            startPosition = StartPosition.Center;
        }

        AutoRoutine routine = this.routineChooser.getSelected();
        if (routine == null)
        {
            routine = AutoRoutine.None;
        }

        this.logger.logString(LoggingKey.AutonomousSelection, startPosition.toString() + "." + routine.toString());

        if (routine == AutoRoutine.PathA)
        {
            return DecidePaths("redPathA", "bluePathA"); 
        }
        else if (routine == AutoRoutine.PathB)
        {
            return DecidePaths("redPathB", "bluePathB"); 
        }
        else if (routine == AutoRoutine.Slalom)
        {
            return Move("slalom");
        }
        else if (routine == AutoRoutine.Barrel)
        {
            return Move("barrelRace");
        }
        else if (routine == AutoRoutine.Bounce)
        {
            return BouncePath("bounce1", "bounce2", "bounce3", "bounce4");
        }
        else if (routine == AutoRoutine.ShootAndTrenchShoot && startPosition == StartPosition.Right) 
        {
            return ShootAndMove("rightShootTrenchShootInator", "rotate180");
        }
        else if (routine == AutoRoutine.ShootAndTrenchShoot && startPosition == StartPosition.Center) 
        {
            return ShootAndMove("centerShootTrenchShoot", "rotate180");
        }
        else if (routine == AutoRoutine.ShootAndShieldShoot && startPosition == StartPosition.Center) 
        {
            return ShootAndMove("centerShootShieldShoot", "rotate1808");
        }
        else if (routine == AutoRoutine.ShootAndMove) 
        {
            return Shoot("doofenshmirtzinator4ft");
        }
        else if (routine == AutoRoutine.Move) 
        {
            return Move("doofenshmirtzinator4ft");
        }
        else if (routine == AutoRoutine.ShootAndShieldShoot && startPosition == StartPosition.Left) 
        {
            return ShootAndMove("leftToOpTrench", "rotate1808");
        }
        else if (routine == AutoRoutine.Shoot) 
        {
            return JustShoot();
        }

        return new PositionStartingTask(0.0, true, true);
    }

    /**
     * Gets an autonomous routine that does nothing
     */
    private static IControlTask GetFillerRoutine()
    {
        return new WaitTask(0.0);
    }

    /**
     * nowwww it's time to get funky
     */
    private static IControlTask DecidePaths(String redPath, String bluePath)
    {
        return SequentialTask.Sequence(
            // new PositionStartingTask(0.0, true, true),
            new IntakePositionTask(true),
            ConcurrentTask.AllTasks(
                new IntakeOuttakeTask(15.0, true),
                new VisionPowercellDecisionTask( 
                    new FollowPathTask(bluePath),
                    new FollowPathTask(redPath))
            )
        );
    }

    private static IControlTask BouncePath(String bounce1, String bounce2, String bounce3, String bounce4)
    {
        return SequentialTask.Sequence(
            // new PositionStartingTask(0.0, true, true),
            new FollowPathTask(bounce1),
            new FollowPathTask(bounce2),
            new FollowPathTask(bounce3),
            new FollowPathTask(bounce4)
        );
    }

    private static IControlTask ShootAndMove(String goToPowerCell, String rotate)
    {
        return SequentialTask.Sequence(
            new PositionStartingTask(0.0, true, true),
            new VisionCenteringTask(),
            new DriveTrainFieldOrientationModeTask(true),
            ConcurrentTask.AnyTasks(
                new FlywheelFixedSpinTask(0.45, 5.0),
                new FullHopperShotTask()),
            ConcurrentTask.AllTasks(
                new FollowPathTask(goToPowerCell),
                new IntakePositionTask(true),
                new IntakeOuttakeTask(9, true)),
            new FollowPathTask(rotate),
            ConcurrentTask.AllTasks(
                new VisionCenteringTask(),
                new FlywheelVisionSpinTask()),
            new FullHopperShotTask()
        );
    }

    // shoot and move off initiation line
    private static IControlTask Shoot(String path)
    {
        return SequentialTask.Sequence(
            new PositionStartingTask(0.0, true, true),
            new VisionCenteringTask(),
            ConcurrentTask.AnyTasks(
                new FlywheelFixedSpinTask(0.45, 5.0),
                SequentialTask.Sequence(
                    new DriveTrainFieldOrientationModeTask(true),
                    new FullHopperShotTask())),
            new FollowPathTask(path));
    }

    private static IControlTask JustShoot()
    {
        return SequentialTask.Sequence(
            new PositionStartingTask(0.0, true, true),
            new VisionCenteringTask(),
            ConcurrentTask.AnyTasks(
                new FlywheelFixedSpinTask(0.45, 5.0),
                SequentialTask.Sequence(
                    new DriveTrainFieldOrientationModeTask(true),
                    new FullHopperShotTask())));
    }

    private static IControlTask Move(String path)
    {
        return SequentialTask.Sequence(
            //new PositionStartingTask(0.0, true, true),
            new FollowPathTask(path));
    }
} // yaaaaaAAAaaaAaaaAAAAaa








































































































































/*
                                      .                                                             
                                    .;+;+                                                           
                                    .+;;'   `,+'.                                                   
                                    ;';;+:..`` :+'+                                                 
                                    ,'+`    .+;;;;;+                                                
                                     ;,,, .+;;;;;'+++;                                              
                                     ;' `+;;;;;#+'+'+''#:.                                          
                                     '`+';;;'+;+;+++'''+'.                                          
                                     #';;;;#';+'+'''+''+'                                           
                                     ;;;;#;,+;;+;;;'''''':                                          
                                     ';'++'.`+;;'';;''+'',                                          
                                     :#'#+'``.'+++'#++'':`                                          
                                      `';++##```##+.''.##                                           
                                      +++#   #`#  `++++                                             
                                      +'#+ # :#: # ##'+                                             
                                      `#+#   +`+   #'#`                                             
                                       :,.+,+,`:+,+..,                                              
                                       `,:```,`,`.`;,                                               
                                        :+.;``.``;.#;                                               
                                        .'``'+'+'``'.                                               
                                         ,````````..                                                
                                          :```````:                                                 
                                          +``.:,``'                                                 
                                          :```````:                                                 
                                           +`````+                                                  
                                            ';+##                                                   
                                            '```'                                                   
                                           `'```'`                                                  
                                         .+''''''''                                                 
                                        +;;;;;;;;''#                                                
                                       :       `   `:                                               
                                      `,            '                                               
                                      +              '                                              
                                     ,;';,``.``.,,,:;#                                              
                                     +;;;;;;;;;;;;;;;'                                              
                                    ,';;;;;;;;;;;;;;;',                                             
                                    +:;;;;;;';;;;;;;;;+                                             
                                   `.   .:,;+;;:::;.``,                                             
                                   :`       #,       `.`                                            
                                   +       # ;        .;                                            
                                  .;;,`    ,         `,+                                            
                                  +;;;;;;''';;;;;;;';;';                                            
                                  +;;;;;;;';;;;;;;;;;'';;                                           
                                 `';;;;;;';;;;;;;;;;;';;+                                           
                                 + `:;;;;+;;;;;;;;';'''::                                           
                                 '     `:  ```````    ,  ,                                          
                                :       '             ;  +                                          
                                '`     ..             ,  ,                                          
                               ,;;;;;..+,`        ```.':;',                                         
                               +;;;;;;'+;;;;;;;;;;;;;;+;;;+                                         
                               ';;;;;;++;;;;;;;;;;;;;;';;;+                                         
                              `.:';;;;;#;;;;;;;;;;;;;;';;;;`                                        
                              ;    `,; ',:;;';;';;;;;:;``  +                                        
                              +      ; ;              ;    `                                        
                              ;      : +              '    `;                                       
                              ';:`` `` '              :`,:;;+                                       
                             `';;;;'+  +,..```````..:;#;;;;;;.                                      
                             `;;;;;;+  +;;;;;;;;;;;;;':';;;;;#                                      
                             .;;;;;;+  ';;;;;;;;;;;;;;,';;;;` .                                     
                             : `.;;'+  +;;;;;;;;;;;;;','.`    +                                     
                             '      ;  +.,,;:;:;;;,..`: ,     ``                                    
                             +      ,  '              : ;   .;'+                                    
                             +.`   ``  +              ;  ;:;;;;':                                   
                             ';;;';;`  +             .'  ;;;;;;;+                                   
                             ';;;;;'   :+++#++##+#+''',   +;;;;.`.                                  
                             +;;;;;'   +;;::;;;+:+;;'',   ,;;.   +                                  
                            ``:;;;;+   +;;:;;;:+;+;;++;    +     .`                                 
                             `   ``'   +;;;;;;;+;+;;'+;     ,   ;#,                                 
                            .      ;   ';;;;;;;;;;;;++'     + .+``.;                                
                            ``     ;   ';;;;;;+;';;;'+'      #`````:,                               
                             +++;,:.   ':;''++;:';:;'';      +``````,`                              
                             ,```,+    +;;';:;;+;;;;'';      +``````,+                              
                            .``````:   ;:;;++';;;;;;';,      ,``:#``+`.                             
                            ,``````'   `';;;;:;;;;;;+;`     '+``+:'`..'                             
                            ,``````'    +;;;;;;;;;;;''     ;:'``#;;.`++                             
                            ```````;    `;:;;;;;;;;;;#     ':'``++:+`+;                             
                            ```'`.`;     +;;;;;;;;;;;+    :::#``' +#`';                             
                            ,``'`:`#     `';;;;;;;;;;+    +:'.`,. ++`;;                             
                            +`.``+`'     :#;;;;;;;;;;;`   +:# ,`  +;`.'                             
                           ,.`+`.:.      ##;;;;;;;;;;;'   ,'`     ;:+#                              
                           '`;.`+`#      ##+;;;;;;;;;;+          ,::;                               
                           ,+,`:``,     :###;;;;;;;;;:'          +:;`                               
                            '`,,`+      ';##';;;;;;;;;;.         +:#                                
                             '+.+       +;;##;;;;;;;;;;'         ;:;                                
                               `       :;;;+#;;;;;;;;;;+        ;::`                                
                                       +;;;;#+;;;;;;;;;;        +:'                                 
                                       ';;;;+#;;;;;;;;;;.       ;:'                                 
                                      ,;;;;;;#;;;;;;;;;;+      +::.                                 
                                      +;;;;;;'';;;;;;;;;'      +:+                                  
                                     `;;;;;;;;#;;;;;;;;;;`    `;:+                                  
                                     ,;;;;;;;;+;;;;;;;;;;+    ':;,                                  
                                     +;;;;;;;;;+;;;;;;;;;'    +:+                                   
                                    .;;;;;;;;;+,;;;;;;;;;;`   ;;+                                   
                                    ';;;;;;;;;, ';;;;;;:;;,  +;:,                                   
                                    ';;;;;;;;'  +;;;;;;;;;'  +:+                                    
                                   ;;;;;;;;;;+  ,;;;;;;;;;+  ;:'                                    
                                   +;;;;;;;;;    ';;;;;;;;;`;:;`                                    
                                   ;;;;;;;;;+    +;;;;;;;;;+#:+                                     
                                  ';;;;;;;;;:    ;;;;;;;;;;';:'                                     
                                 `';;;;;;;:'      ';;;;;;;;;;:.                                     
                                 .;;;;;;;;;+      +;;;;;;;;;'+                                      
                                 +;;;;;;;;;       ';;;;;;;;;#+                                      
                                `;;;;;;;;;+       `;;;;;;;;;;`                                      
                                +;;;;;;;;;.        +;;;;;;;;;`                                      
                                ';;;;;;;:'         ;;;;;;;;;;;                                      
                               :;;;;;;;;;:         `;;;;;;;;;+                                      
                               +;;;;;;;;;           ';;;;;;;;;`                                     
                               ;;;;;;;;;+           ';;;;;;;;;:                                     
                              ';;;;;;;;;;           ,;;;;;;;;;+                                     
                              ':;;;;;;;'             +;;;;;;;;;                                     
                             .;:;;;;;;;'             +;;;;;;;;;:                                    
                             +;;;;;;;;;`             .;;;;;;;;;+                                    
                            `;;;;;;;;;+               ;:;;;;;;;;`                                   
                            ;;;;;;;;;;.               +;;;;;;;::.                                   
                            ';;;;;;;;'`               :;;;;;;;;:+                                   
                           :;;;;;;;;:'                ';;;;;;;;;'                                   
                           ';;;;;;;;'`                +#;;;;;;;;;`                                  
                          `;;;;;;;;;+                 '';;;;;;;;;+                                  
                          +;;;;;;;;;.                '::;;;;;;;;;+                                  
                          ;;;;;;;;;+                 #:'';;;;;;;;;`                                 
                         .#;;;;;;;;'                `;:+;;;;;;;;;;;                                 
                         ':'';;;;;;                 '::.,;;;;;;;;;+                                 
                        +::::+';;;+                 ':'  +:;;;;;;;;`                                
                       `;;;::::;#+:                `;:+  +;;;;;;;:;;      '#+,                      
                       +#::::::::;'`               +:;,  `;;;;:;;'#';;;;;::;:'`                     
                      ';:''::::::::#`              +:'    ';:;;+'::;;:;::::::''                     
                      #+::;+':::::::'.            .:;+    '''+;::;:;:::;:::;':'                     
                    `';+';;:;'';:::::':    '      +::.     +:::::::::::::;#;:#                      
                    :+;#'.''##;#;:;;:::'+  #     `+;'      ;:;::::::::;'+;:'+                       
                   '#;+". ` `+:;+:;::;::+'#+     +:;#     ';:::;:+#+';:::+.                        
                   ';#''      ,+::+#';::;+'#+    ';::      #:;;'+';'''++:`                          
                                '':::;'''#+     ,:;;`      #';:;;:+                                 
                                 `:'++;;':       :++       .;;:;;#,                                 
                                       `                    '':``                                   


*/
