std::vector<Move> waitForMovesFromPC() {
    gpio_put(rowPins[7], 1);
    for (int j = 0; j < numCols; j++) {
        gpio_put(colPins_light[j], 0);    
    }

    char inputBuffer[256];
    while (true) {
        int index = 0;
        // block until newline…
        while (true) {
            int ch = getchar();
            if (ch == '\n' || ch == EOF) break;
            if (index < sizeof(inputBuffer)-1) {
                inputBuffer[index++] = (char)ch;
            }
        }
        inputBuffer[index] = '\0';
        if (index > 0) {
            gpio_put(rowPins[7], 0);
            for (int j = 0; j < numCols; j++) {
                gpio_put(colPins_light[j], 1);    
            }
            // parse and return immediately
            return parseMoves(inputBuffer);
        }
        sleep_us(500);
    }
}
