# Piano Arm MQP

We are looking to construct an anthropomorphic robotic hand that is capable of playing the piano in a human-like manner while coupled with an industrial robotic arm. We hope to be able to demonstrate a performance in which the robot can play a certain segment of a classical piece of keyboard repertoire in a manner similar to that of a human.  In addition, we are looking to study the decisions that humans make when interpreting a piece of music, and build an effective algorithm that can mirror the emotionally expressive nature of human musical performance as realistically as possible.

### Design Requirements

1. **Musical Reproduction**

2. 1. Given an existing industrial robotic arm, we will construct an appendage that resembles a human hand. This hand ought to be able to play keys on a standard piano keyboard, being able to span 61-keys (given the industrial arm we are using, this is the maximum span we can achieve), such that it can play high melodic lines as well as low bass lines. 
   2. Our robotic arm and hand can play notes on a piano keyboard based on a given musical data as input (e.g. MIDI data). This means that it must be able to reproduce rhythms and notes accurately, and adequately plan on how to move along the keyboard when needed. Motion planning work in a similar manner is described by Li & Lai, taking into account the timing offsets needed when shifting the arm to a new position. 
   3. Our robot should be able to play up to 5 notes at the same time, as the human hand has 5 fingers and can play 5 notes in that manner. Therefore, it need not be able to play every single note on the keyboard at the same time. Similar to a human arm, it is limited in the number of notes it can play simultaneously. Thus, we will be able to effectively filter out notes that we cannot play at a given time, and decide which notes to prioritize.
   4. Our robot should be able to play passages that are fast in nature, meaning it can play notes in rapid succession. Humans can play certain passages with great speed and precision, so we should be able to at least match the dexterity of human motion. Franz Liszt’s La Campanella Etude is known for its incredibly fast and animated character. This piece features 64th-note runs at an Allegretto tempo. If we consider this tempo to be roughly 120 beats-per-minute, there are 1,920 fast notes per minute, which is 32 per second. It is reasonable to assume that a concert pianist would not be asked to play faster than this. Such repertoire is known for being at the edge of human capability. Thus it is reasonable for us to say that our robot should be able to play at a rate of 32 notes per second, given that it remains in one position. Of course jumping to other areas of the keyboard will yield a slight delay, so this requirement need not hold in that case.


1. **Human-Like Performance**

2. 1. The robotic arm can replicate motion of a human pianist using the robotic arm in a manner that is expressive and accurate with respect to the gestures of a pianist. If performing alongside a human, there should be a rough correspondence in what kinds of physical gestures are produced. Even though there would be differences between any two human interpretations, there are a number of common patterns that human pianists will tend to follow when playing a passage, as found by Hadjakos et al. 
   2. The robot’s physical gesture should correspond with its musical expressivity. The robot can vary this motion based on the velocity/duration/articulation of notes, along with anticipating motions it should make to play upcoming notes. This means that if we ask the robot to play a certain note with a certain volume, it would adjust its physical approach to playing that note differently than if we asked for a different volume. This includes the motion before and after that note is played. 
   3. The robot can make decisions regarding its physical motions, and the parameters of each note (such as volume, duration, etc.) even if it is given very limited information on what to play. If we limit the amount of information we provide, such as omitting the specific information on how loud each note should be, the robot can actively make decisions on how loud each note should be. This will be based on musical convention, but also contain a certain level of freedom and unpredictability that would come from a human performance. Specifically, we are looking at the volume/velocity at which we play each notes, and the duration of each note/articulation of the music. 
