package frc.robot.driver;

import com.google.inject.Inject;
import com.google.inject.Singleton;

import frc.robot.LoggingKey;
import frc.robot.common.LoggingManager;
import frc.robot.common.robotprovider.*;
import frc.robot.driver.common.*;
import frc.robot.driver.controltasks.*;

@Singleton
public class AutonomousRoutineSelector {
    private final ILogger logger;

    private final PathManager pathManager;

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

        INetworkTableProvider networkTableProvider = provider.getNetworkTableProvider();

        this.routineChooser = networkTableProvider.getSendableChooser();
        this.routineChooser.addDefault("None", AutoRoutine.None);
        networkTableProvider.addChooser("Auto Routine", this.routineChooser);

        this.positionChooser = networkTableProvider.getSendableChooser();
        this.positionChooser.addDefault("center", StartPosition.Center);
        this.positionChooser.addObject("left", StartPosition.Left);
        this.positionChooser.addObject("right", StartPosition.Right);
        networkTableProvider.addChooser("Start Position", this.positionChooser);

        this.generateDynamicPaths(provider.getTrajectoryGenerator());
    }

    /**
     * Check what routine we want to use and return it
     * 
     * @return autonomous routine to execute during autonomous mode
     */
    public IControlTask selectRoutine()
    {
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
        if (routine == AutoRoutine.PathA)
        {
            return SearchPaths( "crissCross", "slideToTheLeft", "slideToTheRight", "forward5ft",
                "forward5ft", "slideToTheRight", "slideToTheLeft", "crissCross"); 
        }
        if (routine == AutoRoutine.PathB)
        {
            return SearchPaths( "crissCross", "slideToTheLeftB", "slideToTheRightB", "chaChaNowYall",
                "forward5ft", "slideToTheRightB", "slideToTheLeftB", "chaChaRealSmooth"); 
        }

        this.logger.logString(LoggingKey.AutonomousSelection, startPosition.toString() + "." + routine.toString());

        return AutonomousRoutineSelector.GetFillerRoutine();
    }

    /**
     * Gets an autonomous routine that does nothing
     */
    private static IControlTask GetFillerRoutine()
    {
        return new WaitTask(0);
    }

    /**
     * Generate any ad-hoc paths and add them to the mapping
     * @param trajectoryGenerator to help generate trajectories
     */
    private void generateDynamicPaths(ITrajectoryGenerator trajectoryGenerator)
    {
        if (trajectoryGenerator == null)
        {
            return;
        }

        this.pathManager.addPath(
            "turnArcLeft",
            trajectoryGenerator.generateTrajectory(
                new Pose2d(0.0, 0.0, 0.0),
                new Pose2d(100.0, 100.0, 90.0),
                new Point2d[]
                {
                    new Point2d(60.0, 60.0)
                }));
        
        // -------------------- path A paths ----------------   

        this.pathManager.addPath(
            "forward5ft",
            trajectoryGenerator.generateTrajectory(
                new Pose2d(0.0, 0.0, 0.0),
                new Pose2d(60.0, 0.0, 0.0), // which way is forward?
                new Point2d[]
                {
                    new Point2d(30.0, 0.0) // is this needed?
                }));
                    
        // D5 TO A6, E6 TO B7
        this.pathManager.addPath( 
            "slideToTheLeft",
            trajectoryGenerator.generateTrajectory(
                new Pose2d(0.0, 0.0, 0.0),
                new Pose2d(30.0, 90.0, 0.0),
                new Point2d[]
                {
                    new Point2d(15.0, 45.0)
                }));

        // C3 TO D5, B7 TO C9 
        this.pathManager.addPath( 
            "slideToTheRight",
            trajectoryGenerator.generateTrajectory(
                new Pose2d(0.0, 0.0, 0.0),
                new Pose2d(60.0, -30.0, 0.0),
                new Point2d[]
                {
                    new Point2d(30.0, -15.0)
                }));
        
        // A6 TO A11, E1 TO E6
        this.pathManager.addPath( 
            "crissCross",                // EVERYBODY CLAP YOUR HANDS  
            trajectoryGenerator.generateTrajectory( // clap
                new Pose2d(0.0, 0.0, 0.0),          // clap
                new Pose2d(150.0, 0.0, 0.0),        // clap
                new Point2d[]                       // clap
                {                                   // clap
                    new Point2d(75.0, 0.0)          // clap
                }));                                // clap



        // ----------------------- Path B paths ------------------  // SLIDE AT YOUR OWN RISK

        // B1 TO B3: forward5feet
        
        this.pathManager.addPath( //B3 TO D5, B8 TO D10
            "slideToTheRightB",
            trajectoryGenerator.generateTrajectory(
                new Pose2d(0.0, 0.0, 0.0),
                new Pose2d(60.0, -60.0, 0.0),
                new Point2d[]
                {
                    new Point2d(30.0, -30.0)
                }));

        this.pathManager.addPath( // D5 TO B7
            "slideToTheLeftB",
            trajectoryGenerator.generateTrajectory(
                new Pose2d(0.0, 0.0, 0.0),
                new Pose2d(60.0, 60.0, 0.0),
                new Point2d[]
                {
                    new Point2d(30.0, 30.0)
                }));

        this.pathManager.addPath( //D10 to D11, 
            "chaChaNowYall", // go forward a lil bit - it'll be a smol amount, almost as smol as VaruANsShiHIKA 
            trajectoryGenerator.generateTrajectory(
                new Pose2d(0.0, 0.0, 0.0),
                new Pose2d(30.0, 0.0, 0.0),
                new Point2d[]
                {
                    new Point2d(15.0, 0.0)
                }));
        
        this.pathManager.addPath( //B7 to B11
            "chaChaRealSmooth", // goes forward 10 ft
            trajectoryGenerator.generateTrajectory(
                new Pose2d(0.0, 0.0, 0.0),
                new Pose2d(120.0, 0.0, 0.0),
                new Point2d[]
                {
                    new Point2d(60.0, 0.0)
                }));
            
    } // one hop this time

    /**
     * nowwww it's time to get funky
     */
    private static IControlTask SearchPaths(String forwardPathB, String secondPathB, String thirdPathB, String finalPathB,
                                            String forwardPathR, String secondPathR, String thirdPathR, String finalPathR)
    {
        return new VisionPowercellTask( // add intake part when we have intake code
            SequentialTask.Sequence( 
            new FollowPathTask(forwardPathB), // FollowPathTask doesn't exist yet
            new FollowPathTask(secondPathB),
            new FollowPathTask(thirdPathB),
            new FollowPathTask(finalPathB)
            ),
            SequentialTask.Sequence( 
            new FollowPathTask(forwardPathR), // FollowPathTask doesn't exist yet
            new FollowPathTask(secondPathR),
            new FollowPathTask(thirdPathR),
            new FollowPathTask(finalPathR)
            )
        ); // first four parameters are the four blue path parts, next four are the four red path parts
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
