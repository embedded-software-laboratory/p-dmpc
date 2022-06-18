This file is used to record the update events
Format: date, author
Example: 2022-06-19, Jianye

# 2022-06-19, Jianye
- Turned `get_road_data.m` to a class called `RoadData`, while the functionality is unchanged, but returns a object of this class containing all the necessary road information and some methods are added.
- Created a class called `ControllResultsInfo`, which is used to initialize the variable `info` which is used to store the results of sub-controller.
- Improved the fallback function called `pb_controller_fallback.m`
    - Before improvement: one vehicle triggers fallback, all other vehicles must take fallback
    - After improvement: only the vehicle triggering fallback and other vehicles which have directed or undirected couplings with this vehicle will take fallbacks. All other vehicles plan trajectory in a normal way.

# 2022-?-?, ?
- ...
- ...
- ...
