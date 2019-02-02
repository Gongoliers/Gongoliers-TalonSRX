# Gongoliers TalonSRX

A library to simplify the use of a TalonSRX in FRC.

View the [JavaDocs](https://gongoliers.github.io/Gongoliers-TalonSRX).

### Features
- Simple configuration of Talon SRX
- Run Talon SRX in several modes (percent output, position, velocity, current)

## Installation
To use Gongoliers TalonSRX with Gradle projects, you can use [JitPack](https://jitpack.io/) by adding the following lines to your `build.gradle` file:

```Gradle
repositories {
    ...
    maven { url 'https://jitpack.io' }
}

dependencies {
    ...
    compile 'com.github.Gongoliers:Gongoliers-TalonSRX:v1.0.0'
}
```

## Usage
```Java
// Create a talon
GTalonSRX talon = new GTalonSRX(0);

talon.set(0.5); // percent output
talon.setCurrent(20); // amps

talon.setPID(0.2, 0, 0, 1); // Set the PID
talon.setSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
talon.setPosition(12); // position
talon.setVelocity(1); // velocity

```

## Contributing
Please fork this repo and submit a pull request to contribute. We will review all changes and respond if they are accepted or rejected (as well as reasons, so it will be accepted).

## License
This project is published under the [MIT license](LICENSE).

