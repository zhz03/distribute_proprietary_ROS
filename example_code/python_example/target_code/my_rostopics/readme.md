# 02_rostopics

## How to run

```
# compile the package 
catkin_make --only-pkg-with-deps 02_rostopics
# set up environment
source devel/setup.bash
# run the node
rosrun 02_rostopics publisher_node.py
```

Check the topic in another terminal by:

```
rostopic list
```

You should be able to the following ros topics:

```
/hello/uclamobility
/rosout
/rosout_agg
```

You can also use the subscriber node to check this topic.

Open a new terminal under the current directory:

```
source devel/setup.bash
# run subscriber node
rosrun 02_rostopics subscriber_node.py
```

You should be able to see the following:

```
data: "hello topic 449"
data: "hello topic 450"
data: "hello topic 451"
data: "hello topic 452"
data: "hello topic 453"
data: "hello topic 454"
data: "hello topic 455"
data: "hello topic 456"
data: "hello topic 457"
data: "hello topic 458"
```

