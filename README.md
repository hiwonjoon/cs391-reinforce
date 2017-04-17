# Environment for Reinforcement learning assignment

![env](assets/demo.json.gif)
**This is very crude, experimental implementation.** ~~Therefore, it could have many bugs.~~
Any improvement, such as come up with better OOP model, will be welcome!

## Dependencies

numpy, asciimatics, better_exceptions

Install the dependencies through pip.

```
pip install numpy
pip install asciimatics
pip install better_exceptions
```

## Optional Dependencies

To record a simulation, it includes the [asciicast2gif](https://github.com/pettarin/asciicast2gif) submodule.

If you want to record your simulation, then you need to fullfull dependencies the submodule requires, such as [asciinema](https://asciinema.org/) and more.

(The most hardeset one to fullfill for me was `phantomjs`. Be careful, not to use old version(<2.0)!. It is only compatible with versio higher than 2.0. According to my experience, it was most easy to install via download binary from official phantomjs [website](http://phantomjs.org/)).

## TODO

write some documentation

## Recording simulatioin

You can record simulation result.

```
asciinema rec demo.json
./python env.py
Ctrl+<D>
```
and copy json file to `asciicast2gif` directory. Then,
```
cd asciicast2gif
./asciicast2gif demo.json demo.gif
```

### Examples recorded with learned policy

1. Learend policy for *HonestCar* module

![env](assets/honest.json.gif)

2. Learend policy for *SafeCar* module

![env](assets/safe.json.gif)

3. Learend policy for *CruseCar* module

![env](assets/cruse.json.gif)

4. Learend policy for *FastCar(avoid parked car)* module

![env](assets/park.json.gif)

5. Learend policy for *FastCar(general case)* module

![env](assets/fast.json.gif)

