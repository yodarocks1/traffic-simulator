# Traffic
![Plot with traffic](https://raw.githubusercontent.com/yodarocks1/traffic-simulator/master//plot.png)

The plot, when run, simulates traffic.
The colors represent the number of vehicles on the road (albeit with no respect to the length of the road segment),
as a percentage of the maximum-count road segment.

## Hover events
### Roads
Hover over roads to see their weights (the time in minutes it takes to travel across the node when no traffic is present)

![Road hover tooltip](https://raw.githubusercontent.com/yodarocks1/traffic-simulator/master//hover_edge.png)

### Intersections
Hover over intersections to see their latitude/longitude, and the number of vehicles currently queueing at that intersection.
They also show an icon representing their type (🚦: Traffic Light, 🛑: Stop Sign, ◯: Roundabout )

![Traffic light hover tooltip](https://raw.githubusercontent.com/yodarocks1/traffic-simulator/master//hover_light.png)
![Stop sign hover tooltip](https://raw.githubusercontent.com/yodarocks1/traffic-simulator/master//hover_stop.png)
![Roundabout hover tooltip](https://raw.githubusercontent.com/yodarocks1/traffic-simulator/master//hover_roundabout.png)

Stop signs will also show which sides have stop signs.
`EW` indicates that North-South traffic is free-flowing, while East-West traffic must stop.
`NS` similarly indicates that East-West traffic is free-flowing, while North-South traffic must stop.

### Generators
Hover over generators to see how much traffic is generated from that direction at that intersection.

![Generator hover tooltip](https://raw.githubusercontent.com/yodarocks1/traffic-simulator/master//hover_generator.png)

## Logic
### Roads
Road weight increases depending on how many cars are on it following the equation:
$w=(cars + 20) / 26.25$

### Intersections
#### 🛑 Stop Signs (All-way)
Stop signs allow one car from one direction every 2 seconds, plus all cars that do not collide with it.

For example, if it's the North-bound traffic's turn, then the first North-bound car can go.

If it turns left, then the East-bound car can go *if it is turning right*, the South-bound car can go *if it is turning left*, and the West-bound car can go *if it is turning right*.

If it goes straight, then the East-bound car can go *if it is turning right*, the South-bound car can go *if it is going straight (if the East-bound car didn't turn right) or turning right*, and the West-bound car cannot go.

Etcetera.

*Scenario 1*: Left turn
```
Allowed:
 - EB right
 - SB left
 - WB right
    ✓    ✓
    ↓ SB ↑
@←←   ↘  ╰←←←✓
 EB  ↖   ↘ WB
✓→→→╮  ↖   →→✓
    ↓ NB ↑
    ✓    @
```
*Scenario 2*: Straight
```
Allowed:
 - SB right
 - EB right
 OR
 - SB straight
    ✓    @            ✓    @    
    ↓ SB ↑            ↓ SB ↑    
✓←←←╯    ↑  ←X     ←  ↓    ↑  ←X
 EB      ↑ WB      EB ↓    ↑ WB 
✓→→→╮    ↑  →X    X→  ↓    ↑  → 
    ↓ NB ↑            ↓ NB ↑    
    ✓    @            ✓    @    
```
*Scenario 3*: Right
```
Allowed:
 - 

        
    ↓ SB ↑
 ←←   ↘  ╰←
 EB  ↖     WB
 →       ╭→→→@
    ↓ NB ↑
         @
```

#### 🛑 Stop Signs (Tributary)
Stop signs **can** allow one car from tributaries every 2 seconds, while all cars from the primary roads continue through.

Cars are only allowed to continue through the intersection if they take an action that is not going to collide with present through-traffic.

#### 🚦 Traffic Lights
Traffic lights have phases that can contain the following parts for each direction: `left turns`, `right turns`, `straights`, and `left yields`.
Right yields are always assumed to be on.
No phase can contain phase parts that collide with one another.

These phases are rotated through by the active `PhaseController`.

#### ◯ Roundabouts
Roundabouts let one car through from *each* direction every 2 seconds.

### Generators
Generators generate, on average (using a normal distribution), the given number of cars per minute.

Vehicles' destinations may be any intersection. Each intersection has a weight, increasing the odds of being selected as a destination.

# Run Options
## python w/ env
 - Open a Windows Command Prompt
   - Yes...it unfortunately has to be Windows, unless you want to make your own env that matches what I used
 - Run `env\Scripts\activate`
 - Run `python app.py`
 - Open `localhost:8050`

## Visual Studio
 - Open `Traffic.sln`
 - Press `Start Without Debugging` (or use `Ctrl+F5`)
 - Open `localhost:8050`
<!--stackedit_data:
eyJoaXN0b3J5IjpbNjgyNjg1NTQ5LDEyNzExOTM1OThdfQ==
-->