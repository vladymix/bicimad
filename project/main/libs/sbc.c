#include "sbc.h"
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_log.h"

/*---I2C----*/
#include "driver/i2c.h"
#include <string.h>
#define I2C_NUM I2C_NUM_0 /*!< I2C port 0 */
/*-----------*/

#define TAG "Sbc.c"

 i2c_port_t portI2C = I2C_NUM_1;

void delayms(int ms)
{
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

void loadInfo()
{

    /* Print chip information */
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), %s%s%s%s, ",
           CONFIG_IDF_TARGET,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
           (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
           (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    printf("silicon revision v%d.%d, ", major_rev, minor_rev);
    if (esp_flash_get_size(NULL, &flash_size) != ESP_OK)
    {
        printf("Get flash size failed");
        return;
    }

    printf("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());

    for (int i = 10; i >= 0; i--)
    {
        printf("Restarting in %d seconds...\n", i);
        delayms(1000);
    }
    printf("Restarting now.\n");
    fflush(stdout);
    esp_restart();
}

void chronometer(int ms, char *text)
{

    int _ms = ms;
    int seg = ms / 1000;
    int minutes = seg / 60;
    int hours = minutes / 60;

    if (seg > 0)
    {
        _ms = ms - (seg * 1000);
    }

    if (minutes > 0)
    {
        seg = seg - (minutes * 60);
    }
    if (hours > 0)
    {
        minutes = minutes - (hours * 60);
    }

    char _s[9];
    sprintf(_s, "%02d", seg);
    char _cms[12];
    sprintf(_cms, "%02d", _ms);

    strcpy(text, _s);
    strcpy(text, ".");
    strcpy(text, _cms);
    // text = _s + "."+_cms;

    // strncpy("Fabricio", text, sizeof("Fabricio"));

    printf("Chronometer %s:%s\n", _s, _cms);
    printf(text);

    // sprintf(text, "%d", ms);
}

void i2c_contrast(OLed *dev, int contrast)
{
    i2c_cmd_handle_t cmd;
    int _contrast = contrast;
    if (contrast < 0x0)
        _contrast = 0;
    if (contrast > 0xFF)
        _contrast = 0xFF;

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);
    i2c_master_write_byte(cmd, OLED_CMD_SET_CONTRAST, true); // 81
    i2c_master_write_byte(cmd, _contrast, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM, cmd, 10 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}

void i2c_display_image(OLed *dev, int page, int seg, uint8_t *images, int width)
{
    i2c_cmd_handle_t cmd;

    if (page >= dev->_pages)
        return;
    if (seg >= dev->_width)
        return;

    int _seg = seg + CONFIG_OFFSETX;
    uint8_t columLow = _seg & 0x0F;
    uint8_t columHigh = (_seg >> 4) & 0x0F;

    int _page = page;
    if (dev->_flip)
    {
        _page = (dev->_pages - page) - 1;
    }

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->_address << 1) | I2C_MASTER_WRITE, true);

    i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);
    // Set Lower Column Start Address for Page Addressing Mode
    i2c_master_write_byte(cmd, (0x02 + columLow), true);
    // Set Higher Column Start Address for Page Addressing Mode
    i2c_master_write_byte(cmd, (0x10 + columHigh), true);
    // Set Page Start Address for Page Addressing Mode
    i2c_master_write_byte(cmd, 0xB0 | _page, true);

    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM, cmd, 10 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->_address << 1) | I2C_MASTER_WRITE, true);

    i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_DATA_STREAM, true);
    i2c_master_write(cmd, images, width, true);

    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM, cmd, 10 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}

void i2c_init(OLed *dev)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);
    i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_OFF, true);   // AE
    i2c_master_write_byte(cmd, OLED_CMD_SET_MUX_RATIO, true); // A8
    if (dev->_height == 64)
        i2c_master_write_byte(cmd, 0x3F, true);
    if (dev->_height == 32)
        i2c_master_write_byte(cmd, 0x1F, true);
    i2c_master_write_byte(cmd, OLED_CMD_SET_DISPLAY_OFFSET, true); // D3
    i2c_master_write_byte(cmd, 0x00, true);
    // i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_DATA_STREAM, true);	// 40
    i2c_master_write_byte(cmd, OLED_CMD_SET_DISPLAY_START_LINE, true); // 40
    // i2c_master_write_byte(cmd, OLED_CMD_SET_SEGMENT_REMAP, true);		// A1
    if (dev->_flip)
    {
        i2c_master_write_byte(cmd, OLED_CMD_SET_SEGMENT_REMAP_0, true); // A0
    }
    else
    {
        i2c_master_write_byte(cmd, OLED_CMD_SET_SEGMENT_REMAP_1, true); // A1
    }
    i2c_master_write_byte(cmd, OLED_CMD_SET_COM_SCAN_MODE, true);   // C8
    i2c_master_write_byte(cmd, OLED_CMD_SET_DISPLAY_CLK_DIV, true); // D5
    i2c_master_write_byte(cmd, 0x80, true);
    i2c_master_write_byte(cmd, OLED_CMD_SET_COM_PIN_MAP, true); // DA
    if (dev->_height == 64)
        i2c_master_write_byte(cmd, 0x12, true);
    if (dev->_height == 32)
        i2c_master_write_byte(cmd, 0x02, true);
    i2c_master_write_byte(cmd, OLED_CMD_SET_CONTRAST, true); // 81
    i2c_master_write_byte(cmd, 0xFF, true);
    i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_RAM, true);       // A4
    i2c_master_write_byte(cmd, OLED_CMD_SET_VCOMH_DESELCT, true); // DB
    i2c_master_write_byte(cmd, 0x40, true);
    i2c_master_write_byte(cmd, OLED_CMD_SET_MEMORY_ADDR_MODE, true); // 20
    // i2c_master_write_byte(cmd, OLED_CMD_SET_HORI_ADDR_MODE, true);	// 00
    i2c_master_write_byte(cmd, OLED_CMD_SET_PAGE_ADDR_MODE, true); // 02
    // Set Lower Column Start Address for Page Addressing Mode
    i2c_master_write_byte(cmd, 0x00, true);
    // Set Higher Column Start Address for Page Addressing Mode
    i2c_master_write_byte(cmd, 0x10, true);
    i2c_master_write_byte(cmd, OLED_CMD_SET_CHARGE_PUMP, true); // 8D
    i2c_master_write_byte(cmd, 0x14, true);
    i2c_master_write_byte(cmd, OLED_CMD_DEACTIVE_SCROLL, true); // 2E
    i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_NORMAL, true);  // A6
    i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_ON, true);      // AF

    i2c_master_stop(cmd);

    esp_err_t espRc = i2c_master_cmd_begin(I2C_NUM, cmd, 10 / portTICK_PERIOD_MS);
    if (espRc == ESP_OK)
    {
        ESP_LOGI(TAG, "OLED configured successfully");
    }
    else
    {
        ESP_LOGE(TAG, "OLED configuration failed. code: 0x%.2X", espRc);
    }
    i2c_cmd_link_delete(cmd);
}

void initOled(OLed *dev)
{
    // CONFIG_I2C_INTERFACE

    ESP_LOGI(TAG, "INTERFACE is i2c");
    ESP_LOGI(TAG, "CONFIG_SDA_GPIO=%d", dev->_sda);
    ESP_LOGI(TAG, "CONFIG_SCL_GPIO=%d", dev->_slc);
    ESP_LOGI(TAG, "CONFIG_RESET_GPIO=%d", dev->_reset);

    // i2c_master_init
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = dev->_sda,
        .scl_io_num = dev->_slc,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ};

    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM, &i2c_config));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM, I2C_MODE_MASTER, 0, 0, 0));

    if (dev->_reset >= 0)
    {
        // gpio_pad_select_gpio(reset);
        gpio_reset_pin(dev->_reset);
        gpio_set_direction(dev->_reset, GPIO_MODE_OUTPUT);
        gpio_set_level(dev->_reset, 0);
        delayms(50);
        gpio_set_level(dev->_reset, 1);
    }

    dev->_address = I2CAddress;
    dev->_flip = false;

    // ssd1306_init
    dev->_width = 132;
    dev->_height = 64;
    dev->_pages = 8;
    if (dev->_height == 32)
        dev->_pages = 4;

    i2c_init(dev);

    // Initialize internal buffer
    for (int i = 0; i < dev->_pages; i++)
    {
        memset(dev->_page[i]._segs, 0, 128);
    }

    // i2c_contrast

    i2c_contrast(dev, 0xff);
}

// Rotate 8-bit data
// 0x12-->0x48
uint8_t ssd1306_rotate_byte(uint8_t ch1)
{
    uint8_t ch2 = 0;
    for (int j = 0; j < 8; j++)
    {
        ch2 = (ch2 << 1) + (ch1 & 0x01);
        ch1 = ch1 >> 1;
    }
    return ch2;
}

uint8_t ssd1306_copy_bit(uint8_t src, int srcBits, uint8_t dst, int dstBits)
{
    ESP_LOGD(TAG, "src=%02x srcBits=%d dst=%02x dstBits=%d", src, srcBits, dst, dstBits);
    uint8_t smask = 0x01 << srcBits;
    uint8_t dmask = 0x01 << dstBits;
    uint8_t _src = src & smask;
#if 0
	if (_src != 0) _src = 1;
	uint8_t _wk = _src << dstBits;
	uint8_t _dst = dst | _wk;
#endif
    uint8_t _dst;
    if (_src != 0)
    {
        _dst = dst | dmask; // set bit
    }
    else
    {
        _dst = dst & ~(dmask); // clear bit
    }
    return _dst;
}

void ssd1306_show_buffer(OLed *dev)
{
    for (int page = 0; page < dev->_pages; page++)
    {
        i2c_display_image(dev, page, 0, dev->_page[page]._segs, dev->_width);
    }
}

// Flip upside down
void ssd1306_flip(uint8_t *buf, size_t blen)
{
    for (int i = 0; i < blen; i++)
    {
        buf[i] = ssd1306_rotate_byte(buf[i]);
    }
}

void ssd1306_invert(uint8_t *buf, size_t blen)
{
    uint8_t wk;
    for (int i = 0; i < blen; i++)
    {
        wk = buf[i];
        buf[i] = ~wk;
    }
}

void oled_display_image(OLed *dev, int page, int seg, uint8_t *images, int width)
{
    i2c_display_image(dev, page, seg, images, width);
    // Set to internal buffer
    memcpy(&dev->_page[page]._segs[seg], images, width);
}

void oled_display_text(OLed *dev, int line, char *text, bool invert)
{
    int size = strlen(text);
    oled_print_text(dev, line, text, size, invert);
}

void oled_display_clear(OLed *dev, int line)
{
    oled_display_text(dev, line, "                ", false);
}

void oled_print_text(OLed *dev, int page, char *text, int text_len, bool invert)
{
    if (page >= dev->_pages)
        return;
    int _text_len = text_len;
    if (_text_len > 16)
    {
        _text_len = 16;
        printf("Error: Text lenght is:%d, max is 16", text_len);
    }

    uint8_t seg = 0;
    uint8_t image[8];
    for (uint8_t i = 0; i < _text_len; i++)
    {
        memcpy(image, font8x8_basic_tr[(uint8_t)text[i]], 8);
        if (invert)
            ssd1306_invert(image, 8);
        if (dev->_flip)
            ssd1306_flip(image, 8);
        oled_display_image(dev, page, seg, image, 8);
        seg = seg + 8;
    }
}

void oled_clear_screen(OLed *dev, bool invert)
{
    char space[16];
    memset(space, 0x00, sizeof(space));
    for (int page = 0; page < dev->_pages; page++)
    {
        oled_print_text(dev, page, space, sizeof(space), invert);
    }
}

void oled_display_bitmap(OLed *dev, int xpos, int ypos, uint8_t *bitmap, int width, int height, bool invert)
{
    if ((width % 8) != 0)
    {
        ESP_LOGE(TAG, "width must be a multiple of 8");
        return;
    }
    int _width = width / 8;
    uint8_t wk0;
    uint8_t wk1;
    uint8_t wk2;
    uint8_t page = (ypos / 8);
    uint8_t _seg = xpos;
    uint8_t dstBits = (ypos % 8);
    ESP_LOGD(TAG, "ypos=%d page=%d dstBits=%d", ypos, page, dstBits);
    int offset = 0;
    for (int _height = 0; _height < height; _height++)
    {
        for (int index = 0; index < _width; index++)
        {
            for (int srcBits = 7; srcBits >= 0; srcBits--)
            {
                wk0 = dev->_page[page]._segs[_seg];
                if (dev->_flip)
                    wk0 = ssd1306_rotate_byte(wk0);

                wk1 = bitmap[index + offset];
                if (invert)
                    wk1 = ~wk1;
                wk2 = ssd1306_copy_bit(wk1, srcBits, wk0, dstBits);
                if (dev->_flip)
                    wk2 = ssd1306_rotate_byte(wk2);

                ESP_LOGD(TAG, "index=%d offset=%d page=%d _seg=%d, wk2=%02x", index, offset, page, _seg, wk2);
                dev->_page[page]._segs[_seg] = wk2;
                _seg++;
            }
        }
        vTaskDelay(1);
        offset = offset + _width;
        dstBits++;
        _seg = xpos;
        if (dstBits == 8)
        {
            page++;
            dstBits = 0;
        }
    }
    ssd1306_show_buffer(dev);
}

// ******* ADC **********

void initAdc2(AnalogicDevice *device)
{
    adc2_config_channel_atten(device->channel, device->adc_atten);
    gpio_num_t adc_gpio_num;
    esp_err_t r = adc2_pad_get_io_num(device->channel, &adc_gpio_num);
    assert(r == ESP_OK);

    ESP_LOGI(TAG, "ADC2 Channel:%d Atten:%d Gpio_num:%d\n", device->channel, device->adc_atten, adc_gpio_num);
}

int readAdc2Value(AnalogicDevice *device)
{
    int read_raw = -1;
    esp_err_t r = adc2_get_raw(device->channel, device->adc_bits_width_t, &read_raw);
    if (r == ESP_OK)
    {
        return read_raw;
    }
    else if (r == ESP_ERR_TIMEOUT)
    {
        printf("Timeout read device.\n");
    }
    else
    {
        printf("Error read");
    }
    return -1;
}

void initAdc1(AnalogicDevice *device)
{
    adc1_config_width(device->adc_bits_width_t);
    adc1_config_channel_atten(device->channel, device->adc_atten);

    gpio_num_t adc_gpio_num;
    esp_err_t r = adc1_pad_get_io_num(device->channel, &adc_gpio_num);
    assert(r == ESP_OK);
    printf("Initialized adc1 channel:%d. adc_atten:%d adc_gpio_num:%d\n", device->channel, device->adc_atten, adc_gpio_num);
}

int readAdc1Value(AnalogicDevice *device)
{
    int val = adc1_get_raw(device->channel);
    return val;
}

void initBMP(BMP280 *dev)
{
    ESP_LOGI(TAG, "INTERFACE is i2c");
    ESP_LOGI(TAG, "CONFIG_SDA_GPIO=%d", dev->_sda);
    ESP_LOGI(TAG, "CONFIG_SCL_GPIO=%d", dev->_slc);
    ESP_LOGI(TAG, "CONFIG_RESET_GPIO=%d", dev->_reset);
    ESP_LOGI(TAG, "CONFIG_ADDRESS=%d", dev->_address);
   


    // i2c_master_init
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = dev->_sda,
        .scl_io_num = dev->_slc,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_100kHZ};

    ESP_ERROR_CHECK(i2c_param_config(portI2C, &i2c_config));
    ESP_ERROR_CHECK(i2c_driver_install(portI2C, I2C_MODE_MASTER, 0, 0, 0));


    esp_err_t espRc;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev->_address << 1) | I2C_MASTER_WRITE, true);

    uint8_t reg_addr = BME280_CHIP_ID_REG;
    uint8_t reg_data = 0x0;

	i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write(cmd, &reg_data, BME280_GEN_READ_WRITE_DATA_LENGTH, true);
	i2c_master_stop(cmd);

	espRc = i2c_master_cmd_begin(portI2C, cmd, 10/portTICK_PERIOD_MS);
	if (espRc == ESP_OK) {
		 ESP_LOGI(TAG, "Sensor configured successfully");
	} else {
		 ESP_LOGE(TAG, "Sensor configuration failed. code: 0x%.2X", espRc);
	}
	i2c_cmd_link_delete(cmd);
}

void readBus(BMP280 dev, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt){
   
	esp_err_t espRc;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev._address << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, reg_addr, true);

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev._address << 1) | I2C_MASTER_READ, true);

	if (cnt > 1) {
		i2c_master_read(cmd, reg_data, cnt-1, I2C_MASTER_ACK);
	}
	i2c_master_read_byte(cmd, reg_data+cnt-1, I2C_MASTER_NACK);
	i2c_master_stop(cmd);

	espRc = i2c_master_cmd_begin(portI2C, cmd, 10/portTICK_PERIOD_MS);
	if (espRc == ESP_OK) {
		 ESP_LOGI(TAG, "Sensor readed");
	} else {
		 ESP_LOGE(TAG, "Sensor fail readed");
	}

	i2c_cmd_link_delete(cmd);
}
