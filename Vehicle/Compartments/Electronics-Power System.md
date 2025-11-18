# For our power system we chose: Li-ion batteries
Firstly, we compared the characteristics of Lithium, Lithium-Ion, Lipo and Alkaline batteries

### Lithium batteries
>Lithium batteries are one of the most commonly used battery types. They offer the highest energy density of any other battery cell, meaning they store more energy than other batteries, such as alkaline. Lithium batteries are only sold in AA, AAA, and 9V sizes; however, their mAh ratings exceed every other non-rechargeable battery. One AA lithium battery ranges from 2,700-3,400 mAh, and will last a long time, even under heavy-use.
<ins> Pros </ins>
<ins>**Last up to 4x longer compared to other battery types
Light-weight for portable devices
Ideal for heavy-use**</ins>
Function well even in extreme temperatures, working well in outdoor devices like flashlights
Shelf life is 10-12 years; great for use in emergency devices, as the batteries will not leak or explode
<ins> Cons </ins>
Much more expensive than other battery types
<ins>**Highly flammable; require special disposal at specified locations and cannot be thrown away
Don’t come in bigger battery sizes like C or D**</ins>

### Lithium-ion batteries 
> Lithium-ion batteries (Li-ion) are <ins>**highly efficient, with a long cycle life, high energy and power density, and fast discharge capabilities**</ins>, making them ideal for portable electronics. They can last up to 15 years, significantly longer than lead-acid batteries, reducing replacement costs. These batteries typically offer 500-7000 discharge cycles before needing replacement, depending on the chemistry used. The specific energy of Li-ion batteries ranges from 100–265 Wh/kg, with an energy density of 250–693 Wh/L. Characterizing the state of charge involves extensive experiments to evaluate parameters like voltage, current, temperature, capacity, and impedance. Li-ion batteries consist of four main components: cathode, anode, electrolyte, and separator, all essential for their function. Unlike lithium cells, <ins>**Li-ion cells are rechargeable, with lifespans averaging 2-3 years or 300-500 charge cycles.** </ins>


### Lipo batteries
>The advantages of Lipo batteries are<ins> **small volume, huge capacity, light weight, non-pollution, high voltage of single cell, low self-discharging rate, more cycle times**</ins>, but its price is comparatively high.
A Lipo battery is constructed from separate cells, all connected to form the specific battery. One Lipo cell has a nominal voltage of 3.7V. When connecting these in series, the voltage increases, meaning you get 7.4V for a 2 cell battery, 11.1V for a 3 cell battery, 14.8V for a 4 cell battery etc.
Enhanced Power Density: LiPo batteries offer a slightly higher power density, which means they can deliver a burst of power more quickly. This characteristic is valuable in scenarios where immediate, high-energy output is necessary.
How long does a 5,000mAh LiPo battery last? About 25–30 minutes on full discharge. Depending on what you are using it for. A batter such as this on a remote RC car being pushed at full throttle will last about 25 minutes.
<ins> **Its chemistry leads to fire when the LiPo battery is punctured. Lithium polymer battery requires special care during charging, discharging, and storage. Li polymer battery is expensive. The cost is almost double that Li-Ion battery.** </ins>
It is commonly accepted that the preferred way to dispose of a model LiPo pack is to immerse it in a container of salt water for an extended period 

### Alkaline batteries
>Alkaline batteries are economical, easy to dispose of, and extremely popular. They normally have a<ins> **capacity rating of over 2,500 mAh**</ins>, great for moderate to heavy-use devices. Unlike lithium batteries, almost every standard size battery offers an alkaline construction, making it perfect for most devices.
<ins> Pros </ins>
Great price for quality
<ins>**Last longer due to potassium hydroxide construction**</ins>
Shelf life is 5-10 years
Function well even in extreme temperatures, working well in outdoor devices like flashlights
Lead, mercury, and cadmium-free is good for the environment, and the batteries don't need to be disposed of in a specific way
<ins> Cons </ins>
<ins>**Heavy and bulky, without additional voltage
May not work well in high drain devices
Will sometimes leak, causing the device to become unusable**</ins>

### In the end we found that:
"If you are looking for the highest voltage under load (punch or top speed) then choose a LiPo. If you are looking for the highest capacity for the weight (energy density), choose Li-ion. If safety is a big concern, then choose Li-ion."
Additionally, they offer us the required voltage, they have a capacity of 2500-3500mAh and are safer than Li-Po
**And therefore we chose <ins>Li-ion</ins> for our vehicle.**

> [!IMPORTANT]
> ### Just because the Li-ion batteries were the best fit for us, doesn't neccessarily mean they'll be the best fit for everyone.
> ### Depending on your builds requirements and needs, you can utilize any of the above batteries as **alternatives**.

*Characteristics*
>The system is powered using three (3) Li-ion 18650 batteries connected in series (3S), that offer us up to 12.6V when fully charged. This layout ensures an abundance of electrical power for the operation of our system.
>
>For **protection** we utilised a Charger Protection Module 3S 10A which offers us:
>- Overcharge protection (Stops from overloading)
>- Over-discharge protection (Interrupts sudden voltage drops that might hurt the batteries)
- Short-circuit protection (As the name suggests, stops the system and batteries in case of short-circuiting)
>This specific module can provide a maximum continuous flow of 6-8A, an amount suitable for safe power distribution to the Raspberry Pi 5 and its peripherals.
>We also used a DC-DC Converter module Step-Down (5V/3A), based on the LM2596S. It has a voltage range of 3.2V - 40V (Input) and 1.3V - 35V output with a maximum of 3A.
>All of the above combined with the small size of the placket make it ideally suitable for our build

*Compatability*
>An external power jack lets us power or charge the robot without opening the chassis or disconnecting the battery, which makes
maintenance much easier

*Requirements*
> We used 3 18650 Li-ion cells in series (3S), a BMS, a main switch, a 4 A fuse and an LM2596 buck converter that generates a stable 5 V rail
> It was necessary to keep the batteries contained in a plastic sealed box when not in use to avoid danger due to flamability.

