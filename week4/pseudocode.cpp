const timespan = 100;
const stepsize = 10;

// Slaap altijd iig de helft van de tijd
sleep(timespan);

int counter = 0;
while(counter < timespan) {
    counter += stepsize;

    if(counter >= timespan) {
        // Stuur een signaal
        radio.send(); // TODO; echte functie gebruiken!

        // Reset de state van de machine
        counter = 0;
    }

    if(radio.available()) {
        counter = timespan - (timespan-counter)/2
    }

    sleep(stepsize);
}
