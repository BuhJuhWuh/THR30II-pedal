# THRII-direct-USB-pedalboard
Buhjuhwuh's fork of martinzw's controller pedal for Yamaha THR30II amp.

A pedalboard that can send complete settings patches (as well as individual parameter settings) to a THRII guitar amplifier with direct USB-connection.

It rests heavily on the excellent work done by martinzw to reverse engineer the communications protocols.  His 6-button pedal can switch between pre-programmed patches (generated from THR app on PC in .thrl6p format), as well as incorporating solo patch and volume-boosting solo modes.  Read up on his project first.

My wishlist:

(done)

<ol>
  <li>Built pedal. With more buttons!</li>
  <li>Included ability to daisy-chain THR's power supply, or run from onboard battery;</li>
  <li>Updated screen libraries to match my hardware;</li>
  <li>Updated button libraries to add multiple functions (tap/hold/double tap);</li>
  <li>Included status lights;</li>
  <li>Included inputs for a pair of volume/expression pedals;</li>
  <li>Can now switch on and off FX units;</li>
  <li>Can now switch between FX modes (this necessitated a move away from standard .thrl6p format, since it doesn't store all FX unit parameters);</li>
  <li>Edited screen UI, replacing text with icons, using colour-coding to show FX unit statuses;</li>
  <li>Edited patch switching logic to make it more to my taste, including ability to change between "select then submit" and "change immediately" modes;</li>
  <li>Replaced solo modes with ability to turn on and off gain boost, gate, and compressor;</li>
  <li>Added ability to switch amp collection/type and cabinet type.</li>
</ol>

(in progress)

<ol>
  <li>Expression pedals are sensed and read correctly - now need to connect inputs to selected parameters, including means of selecting which parameters they are attached to.  This probably requires a new state machine.  Start by making both assignable; may choose to dedicate one to volume only.</li>
</ol>

(to do)

<ol>
  <li>Add "save mode" giving ability to save patches back to SD card (including copying/re-ordering patches, and naming patches via UI);</li>
  <li>Add "parameter edit mode" to allow direct editing from the pedal: view and change individual parameters with expression pedal and/or buttons;</li>
  <li>Add tap tempo input for delay/FX, including different tap tempo modes (crotchet/dotted/triplet/etc);</li>
  <li>Any way to add tuner access (both control and LED feedback) from foot pedal...?</li>
</ol>

<p><img src="/images/THR%20pedal%20front.jpg" alt="THR pedal front" title="THR pedal front" width="800"></p>

| Button (from top left) | Function | Tap | Hold |
| :---: | --- | --- | --- |
| 1 | Patch | Submit | (Patch save mode) |
| 2 | Amp | Cycle sim | Collection > Amp > Cabinet > |
| 3 |  |  |  |
| 4 |  |  |  |
| 5 |  |  |  |
| 6 |  |  |  |
| 7 |  |  |  |
| 8 |  |  |  |
| 9 |  |  |  |
| 10 |  |  |  |

<p><img src="/images/THR%20pedal%20back.jpg" alt="THR pedal back" title="THR pedal back" width="800"></p>
<p>L-R: exp pedal TRS inputs x2, USB-B input from PC, USB-A output to amp, (top) power supply selection (USB/mains), (bottom) on/off, DC power jacks (can daisy chain amp's 15V power supply).</p>
<p><img src="/images/THR%20pedal%20screen.jpg" alt="THR pedal screen" title="THR pedal screen" width="800"></p>

<p><img src="/images/THR%20pedal%20innards.jpg" alt="THR pedal innards" title="THR pedal innards" width="800"></p>

<p><img src="/images/THR%20pedal%20wiring%20as-built.png" alt="THR pedal wiring as-built" title="THR pedal wiring as-built" width="800"></p>

I agree with martinzw's **DISCLAIMER:**

THE HARDWARE SUGGESTIONS THE LISTED SOFTWARE AND ALL INFORMATION HERE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHOR OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE AND HARDWARE SUGGESTIONS OR THE USE OR OTHER DEALINGS IN THE SOFTWARE AND HARDWARE.

I have no connection to Yamaha / Line6. Every knowledge about the data communications protocol was gained by experimenting, try and error and by looking at the data stream. I especially have not decompiled any part of the Yamaha software application nor of the THR's firmware. Described protocol behaviour can be totally wrong. I only describe results from (good) guesses and succeded experiments - not facts!

I can not ensure, that sending messages in the suggested way will not damage the THRII. As well I can not guarantee, that the hardware, i describe will not damage your THRII device or it's power supply or your computer.

