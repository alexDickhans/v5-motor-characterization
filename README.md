# Motor Characterization System

A tool to help you figure out which of your VEX motors are dying and how bad they are. Instead of guessing when to replace motors, this gives you actual data to make smart decisions.

## The Problem

We all know some motors perform worse than others, but it's hard to tell which ones are actually bad and how much worse they are. This tool lets you test any motor and see exactly how it's performing - and track how it gets worse over time.

**Want to help build a database of motor lifespan data?** Contact me on Discord: @alex2654e

## What It Does

1. **Tests your motor** at different speeds
2. **Measures how it responds** (100 times per second)
3. **Gives you three key numbers**:
   - **kS**: How much friction it has (higher = more worn out)
   - **kV**: How efficient it is (lower = less efficient)
   - **kA**: How responsive it is (weird values = problems)
4. **Shows results** on your V5 brain screen
5. **Lets you track changes** over time

## Why Bother?

### **Find Bad Motors**
- **High kS** = lots of friction (probably worn bearings)
- **Low kV** = not very efficient (getting old)
- **Weird kA** = electrical problems

### **Track Wear**
- **Compare numbers** over time
- **Spot motors** that are getting worse
- **Replace before** they completely die

### **Save Money**
- **Replace based on data**, not just age or a feeling
- **Fix problems early** before they break things
- **Keep your robot reliable**

## Quick Start

### Setup
1. Connect a motor to **port 1** (or change it in the code)
2. Make sure it can spin freely
3. Upload the code to your V5 brain

### Use It
1. **Press A** on your controller
2. **Wait 20 seconds** (it's testing the motor)
3. **Write down the numbers** for later
4. **Press A again** anytime to retest

## Tracking Performance

### **First Time (Baseline)**
When you get new motors:
1. Test each one
2. Write down kS, kV, kA, and R²
3. Label them with their "good" numbers

### **Regular Checks** *Not tested yet*
- (ideally) **Every week** for competition robots
- **Before/after big events**

### **Red Flags** *Not tested yet*
- **kS goes up 20%+**: Bearing wear
- **kV goes down 15%+**: Efficiency loss
- **R² below 0.9**: Something's wrong
- **kA goes negative**: Electrical problems

## What the Numbers Mean

### **kS (Static Friction)** *Not tested yet*
- **Good**: 1.5 - 3.0 V
- **Bad**: >4.0 V (worn bearings, binding)
- **Gets worse**: Normal wear over time
- **Suddenly bad**: Something broke

### **kV (Efficiency)** *Not tested yet*
- **Good**: 0.08 - 0.15 V/RPM
- **Bad**: <0.06 V/RPM (old, damaged)
- **Gets worse**: Normal aging
- **Very low**: Time to replace

### **kA (Responsiveness)** *Not tested yet*
- **Good**: 0.0005 - 0.002 V/(RPM/s)
- **Bad**: Negative values (problems)
- **Inconsistent**: Electrical issues
- **Changes**: Less common, but bad

### **R² (How Good the Test Was)** *Not tested yet*
- **> 0.95**: Great test, trust the numbers
- **0.90 - 0.95**: Good test, numbers are reliable
- **0.80 - 0.90**: OK test, check for issues
- **< 0.80**: Bad test, motor might be broken

## Example Results (not taken from real motors)

### **New Motor**
```
kS: 2.1 V    (normal friction)
kV: 0.12 V/RPM    (good efficiency)
kA: 0.001 V/(RPM/s)    (responsive)
R²: 0.98    (excellent test)
```

### **6 Months Later**
```
kS: 3.5 V    (67% more friction - bearings wearing)
kV: 0.09 V/RPM    (25% less efficient - getting old)
kA: 0.001 V/(RPM/s)    (still responsive)
R²: 0.93    (still good test, but declining)
```

### **Needs Replacement**
```
kS: 5.2 V    (way too much friction)
kV: 0.06 V/RPM    (very inefficient)
kA: -0.001 V/(RPM/s)    (negative - electrical problems)
R²: 0.85    (poor test - motor is bad)
```

## Using the Results

### **Compare Motors**
```cpp
// Motor 1: kS: 2.1, kV: 0.12, kA: 0.001, R²: 0.98
// Motor 2: kS: 3.8, kV: 0.09, kA: 0.001, R²: 0.92

// Motor 2 is showing wear:
// - 81% more friction (kS)
// - 25% less efficient (kV)
// - Lower test quality (R²)
```

### **Track Over Time**
```cpp
// Week 1: New motor
// kS: 2.1, kV: 0.12, kA: 0.001, R²: 0.98

// Week 4: After competition
// kS: 2.4, kV: 0.11, kA: 0.001, R²: 0.96

// Week 8: Before replacement
// kS: 3.2, kV: 0.08, kA: 0.001, R²: 0.91

// Analysis: Motor is wearing out
// - 52% more friction (bearing wear)
// - 33% less efficient (aging)
// - Test quality declining (problems)
```

### **Use for Control**
```cpp
// Calculate what voltage to give the motor
double targetSpeed = 100.0; // RPM
double feedforwardVoltage = kS * (targetSpeed > 0 ? 1 : -1) + 
                           kV * targetSpeed + 
                           kA * 0.0; // no acceleration

// Send it to the motor
motor.move_voltage(feedforwardVoltage * 1000);
```

## Files

- `src/main.cpp` - The main code
- `include/system_identification.hpp` - Math stuff
- `src/system_identification.cpp` - More math stuff

## Technical Stuff

- **Tests for 20 seconds** total
- **100 measurements per second**
- **Tests from -12V to +12V**
- **Uses fancy math** (Eigen) to find the numbers

## Common Problems

### **Bad R² (< 0.8)**
- Motor is stuck or binding
- Check for mechanical problems
- **Might mean the motor is broken**

### **Weird Numbers**
- **kS > 5.0 V**: Too much friction, bearings bad
- **kV < 0.05 V/RPM**: Very inefficient, motor dying
- **kA negative**: Electrical problems

### **Test Fails**
- Check motor connection
- Make sure it can spin freely
- Check power supply

## Tips

1. **Test motors alone** - remove other loads
2. **Warm up first** - run motor briefly before testing
3. **Test multiple times** - make sure results are consistent
4. **Write everything down** - build your own database
5. **Compare over time** - track how motors wear
6. **Replace early** - don't wait for complete failure

## Help Build the Database

We're collecting data to understand:
- **How long motors last**
- **How they wear out**
- **When they typically fail**
- **How to predict failures**

**Want to help?**
1. Test your motors regularly
2. Write down the numbers
3. Note motor type, age, and usage
4. Contact @alex2654e on Discord

This will help everyone make better decisions about motor maintenance!

## License

For VEX robotics use. Follow VEX and PROS rules.
