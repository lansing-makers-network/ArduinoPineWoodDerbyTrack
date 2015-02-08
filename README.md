# ArduinoPineWoodDerbyTrack
Lansing Makers Network OpenSource Pinewood Derby track was created for the local cub scouts to practice their speedy creations. The track is available during open hours along with the wood shop to build their racers, when accompanied with a responsible guardian. The derby track system consists primarily of the following components; the Track, the Starting Gate and the Arduino based timing computer system.
 

Initially created as standalone track, it was simply to exercise derby racers that were recently constructed. The track itself primarily consists of 5 pieces of 8’ sections of prefinished ¾” 11 layer cabinet plywood that attach to each other using 4” lap joints, allowing simple re-assembly of a 38’ continuous run. Four lanes are implemented using ¼” Maple plywood stripes as wheel guides for the racers to straddle navigating the racers straight down the track. The breaking runoff was shaped by raising the last 4’ of lane guides another ½” above the cabinet plywood and adhering stripes of 2mm Craft Foam to the surface of the lane guides, assisting as a gentle break for the racers. The use of prefinished cabinet plywood provides a low friction and smooth surface creating a fast derby track. Comparable to that of Aluminum tracks, where the faster racers have been recorded with times as low as 2.8 seconds over its 33’ start to finish line length, most racers average between 3.1 to 3.4 seconds, providing an enjoyable experience.

The starting gate consists of a plywood trap door attached under the elevated starting area with 4” starting pins that protrude through openings in the track and lane guides as to provide a resting stop for each of the racers to be readied against. Activating the latching solenoid releases the trap door, dropping the pins, allowing the racers to begin their gravity race. A sensor on the door indicates when the door has actually dropped and the race has begun. Closing the door will then clear the display results and ready the track for another competition. 
 

The Arduino based timing computer system uses an inexpensive Arduino Mega 2560 that measures active IR reflective object sensors that are embedded into the tracks lane guides, both to determine if a racer is present at the starting gate and when it passes the finish line. Each object sensors has pull up and current limiting resistors directly soldered to the sensors along with its wiring embedded into the underneath of the track, reducing the number of connections to the computer. 
 

Both the Start and Finish sensors are attached using modular RJ45 receptacles, as typically found on telecom wall outlets. Each connection is embedded into the side of the tracks, providing a sturdy and flush connection. 
 
 

The cabling for these connections use readily available CAT-5 patch cables to connect to the display panel, mounted vertically behind the starting gate. The corresponding RJ45 receptacles on the panel have been mounted into angle brackets that have been CNC’ed allowing standard snap-in mounting of the modular connections. Different modular connections, preventing incorrect cabling, implement an RCA connection providing wiring to the starting gate solenoid and an RJ11 connection to a provide Remote Starting Trigger. The Trigger is constructed of elastic phone cord, PVC pipe for a handle and red button. The system may be powered using either a 12Vdc wall supply through a typical 2.5mm DC Jack or battery for operation in remote areas.
 


The display panel has both large Alphanumeric 2.3” digit driven by SPI based HC595 IC’s and smaller Alphanumeric Quad 0.54” Digits driven by I2C based HT16K33 backpack modules over each lane to indicate both the winning order of place and its elapsed time of the race. The digits are mounted into a laser cut 1/8” plywood, providing alignment behind smoke grey transparent acrylic producing a comfortable to read display. The Arduino’s firmware is open source and available on LMN’s Github repo, which additionally uses the alphanumeric digits to provide auxiliary information about status. Making it easier to operate the track, indicating when racers are ready, missing, when the track is not yet ready or when the race is completed.
 
http://youtu.be/da9Q15YSw38

Michael P. Flaga, michael@flaga.net

