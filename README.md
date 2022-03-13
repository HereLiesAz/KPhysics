# KPhysics (based on [JPhysics](https://github.com/HaydenMarshalla/JPhysics))
KPhysics is a Kotlin port of Hayden Marshalla's JPhysics. It is a 2D physics engine with zero third-party dependencies.

The engine is written in Kotlin and has been created with the intention of being used in games.

## Tech demos

A tech demo can be found [here]() and includes various examples to show what the engine is capable of.
![Chains Demo](https://i.postimg.cc/50Ggn2qL/Chains.png "Chains")
![Particle Demo](https://i.postimg.cc/ZKgmp8d5/Particle-explosion-demo.png "Particles")
![Shadow Demo](https://i.postimg.cc/13qQH8Gc/Shadow-casting.png "Shadows")

## Feature List
### Physics
- Rigid body dynamics
- Primitive joint constraints
- Momentum
- Friction
- Restitution
- Collision response (Sequential Impulses Solver)
- Stable object stacking
- Orbits
- Explosions
- Object slicing

### Collision
- AABB queries (Broadphase)
- One-shot contact manifolds
- Discrete collision detection
- Convex polygon and circle collisions
- Ray casting
- Position resolution handling

### Explosion types
- Proximity
- Ray casting
- Particle

## Using KPhysics

### Prerequisites
- An appropriate IDE for example Intellij (with java 1.8+ JDK installed)
- Maven/Gradle or other dependency manager

### Add KPhysics to your project
Add KPhysics to your classpath by adding a Maven/Gradle dependency from [Jitpack](https://jitpack.io/#Chafficui/KPhysics).

#### Maven
```
<repositories>
  <repository>
    <id>jitpack.io</id>
    <url>https://jitpack.io</url>
  </repository>
</repositories>
<dependencies>
  <dependency>
    <groupId>com.github.Chafficui</groupId>
    <artifactId>KPhysics</artifactId>
    <version>Tag</version>
  </dependency>
</dependency>
```

#### Gradle
```
allprojects {
    repositories {
        ...
        maven { url 'https://jitpack.io' }
    }
}
dependencies {
    ...
    compile 'com.github.Chafficui:KPhysics:Tag'
}
```

## Documentation
Coming soon!

## Authors
 - Chafficui

 (based on a JPhysics by HaydenMarshalla)

## License
[License](https://github.com/Chafficui/KPhysics/blob/master/LICENSE)

## Credits
Everything in this project is based on the [JPhysics](https://github.com/HaydenMarshalla/JPhysics).