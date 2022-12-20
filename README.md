# cmpt-417-individual-and-final-project

The following search algorithms for the MAPF problem have been implemented in this project: prioritized search, conflict based search (**CBS**), increasing cost tree search (**ICTS**), and enhanced partial expansions A* (**EPEA***). 

You can test and visualize the code by using:

```
$ python run_experiments.py --instance instances/exp1.txt --solver Independent
```
Replace ```Indepenent``` with desired MAPF solver, replace ```instances/exp1.txt``` for other testing instances.

Test all instances by running:
```
python run_experiments.py --instance "instances/test_*" --solver CBS --batch
```
(This may take a while depending on your computer.) The ```batch``` command creates an output file *results.csv* which contains all the test results.
