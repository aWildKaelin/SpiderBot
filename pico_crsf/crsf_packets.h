typedef struct crsf_rc_channels {
    // 16 channels packed into 22 bytes, 11 bits per channel
    unsigned int channel_1: 11;
    unsigned int channel_2: 11;
    unsigned int channel_3: 11;
    unsigned int channel_4: 11;
    unsigned int channel_5: 11;
    unsigned int channel_6: 11;
    unsigned int channel_7: 11;
    unsigned int channel_8: 11;
    unsigned int channel_9: 11;
    unsigned int channel_10: 11;
    unsigned int channel_11: 11;
    unsigned int channel_12: 11;
    unsigned int channel_13: 11;
    unsigned int channel_14: 11;
    unsigned int channel_15: 11;
    unsigned int channel_16: 11;
} __attribute__ ((__packed__)) crsf_rc_channels;

typedef struct crsf_link_statistics{
    uint8_t     up_rssi_ant1;       // Uplink RSSI Antenna 1 (dBm * -1)
    uint8_t     up_rssi_ant2;       // Uplink RSSI Antenna 2 (dBm * -1)
    uint8_t     up_link_quality;    // Uplink Package success rate / Link quality (%)
    int8_t      up_snr;             // Uplink SNR (dB)
    uint8_t     active_antenna;     // number of currently best antenna
    uint8_t     rf_profile;         // enum {4fps = 0 , 50fps, 150fps}
    uint8_t     up_rf_power;        // enum {0mW = 0, 10mW, 25mW, 100mW,
                                    // 500mW, 1000mW, 2000mW, 250mW, 50mW}
    uint8_t     down_rssi;          // Downlink RSSI (dBm * -1)
    uint8_t     down_link_quality;  // Downlink Package success rate / Link quality (%)
    int8_t      down_snr;           // Downlink SNR (dB)
} crsf_link_statistics;

