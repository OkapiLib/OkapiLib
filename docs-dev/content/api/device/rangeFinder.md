## RangeFinder

### Constructor

```c++
//Signature
RangeFinder(const unsigned char iportTop, const unsigned char iportBottom)
```

Parameter | Description
----------|------------
iportTop | Top digital port
iportBottom | Bottom digital port

### get

```c++
//Signature
int get()
```

Return the current measured range in centimeters.

### getFiltered

```c++
//Signature
int getFiltered()
```

Return the median measured range in centimeters. The median is measured across the previous five measurements.
