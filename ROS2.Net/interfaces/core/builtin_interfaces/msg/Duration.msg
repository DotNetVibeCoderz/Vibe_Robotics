# Duration defines a period between two time points.

# Seconds component, range is valid over any possible int32 value.
int32 sec

# Nanoseconds component in the range of [0, 1e9), to be added to the seconds component.
uint32 nanosec
