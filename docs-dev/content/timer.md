---
weight: 220
title: Timer
---

# Timer

The `Timer` class has timing-related utilities to make measuring time differences and writing non-blocking code in loops easier.

## Constructor

```c++
//Signature
Timer()
```

The constructor does not take any parameters.

## getDt

```c++
//Signature
unsigned long getDt()
```

Return the time passed in ms since the last time this function was called.

## getStartingTime

```c++
//Signature
unsigned long getStartingTime() const
```

Return the time in ms the timer was constructed.

## getDtFromStart

```c++
//Signature
unsigned long getDtFromStart() const
```

Return the time passed in ms since the timer was constructed.

## placeMark

```c++
//Signature
void placeMark()
```

Place a time marker. Placing another marker will overwrite the previous one.

## placeHardMark

```c++
//Signature
void placeHardMark()
```

Place a hard time marker. Placing another hard marker will not overwrite the previous one; instead, `clearHardMark()` must be called before another can be placed.

## clearHardMark

```c++
//Signature
unsigned long clearHardMark()
```

Clear and return the current hard marker.

## getDtFromMark

```c++
//Signature
unsigned long getDtFromMark()
```

Return the time in ms since the marker was placed.

## getDtFromHardMark

```c++
//Signature
unsigned long getDtFromHardMark()
```

Return the time in ms since the hard marker was placed.

## repeat

```c++
//Signature
bool repeat(unsigned long ms)
```

Return true when the input time period `ms` has passed, and then reset. Meant to be used in loops to execute a block of code every so many ms without blocking the loop.

Parameter | Description
----------|------------
ms | Number of milliseconds between calls of `repeat` that return true
