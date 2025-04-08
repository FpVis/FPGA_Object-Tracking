module object_tracker #(
    parameter box_width  = 80,
    parameter box_height = 80
) (
    input logic clk,
    input logic reset,
    input logic pclk,
    input logic frame_done,
    input logic [7:0] detection_threshold,
    input logic diff_detected,

    input logic [9:0] x_pixel,
    input logic [9:0] y_pixel,
    input logic       display_enable,

    output logic box_active
);

    typedef enum {
        IDLE,
        TRACKING
    } state_e;
    state_e state, next_state;

    logic [19:0] sum_x;
    logic [19:0] sum_y;
    logic [16:0] pixel_count;

    logic [ 9:0] curr_center_x;
    logic [ 9:0] curr_center_y;
    logic        curr_motion_valid;

    logic [ 9:0] center_x;
    logic [ 9:0] center_y;

    logic        motion_valid;


    logic [$clog2(12_500_000) - 1 : 0] half_sec_counter;
    logic half_sec_tick;

    always_ff @(posedge clk, posedge reset) begin
        if (reset) begin
            half_sec_counter <= 0;
            half_sec_tick <= 0;
        end else if (half_sec_counter == 12_500_000) begin
            half_sec_counter <= 0;
            half_sec_tick <= 1;
        end else if (state != next_state) begin
            half_sec_counter <= 0;
            half_sec_tick <= 0;
        end else begin
            half_sec_counter <= half_sec_counter + 1;
            half_sec_tick <= 0;
        end
    end


    always_ff @(posedge clk, posedge reset) begin
        if (reset) begin
            state <= IDLE;
        end else begin
            state <= next_state;
        end
    end

    always_comb begin
        next_state = state;
        motion_valid = 0;

        case (state)
            IDLE: begin
                motion_valid = 0;
                if(pixel_count > detection_threshold) begin
                    next_state = TRACKING;
                end
            end


            TRACKING: begin
                motion_valid = 1;
                if(half_sec_tick) begin
                    next_state = IDLE;
                end
            end            
        endcase
    end

    always_ff @(posedge pclk, posedge reset) begin
        if (reset) begin
            sum_x <= 20'b0;
            sum_y <= 20'b0;
            pixel_count <= 17'b0;
        end else begin
            if (frame_done) begin
                sum_x <= 20'b0;
                sum_y <= 20'b0;
                pixel_count <= 17'b0;
            end else if (diff_detected) begin
                if (x_pixel < 320 && y_pixel < 240) begin
                    sum_x <= sum_x + x_pixel;
                    sum_y <= sum_y + y_pixel;
                    pixel_count <= pixel_count + 1;
                end
            end
        end
    end

    always_ff @(posedge frame_done, posedge reset) begin
        if (reset) begin
            curr_center_x <= 10'd0;
            curr_center_y <= 10'd0;
        end else begin
            if (pixel_count > detection_threshold) begin
                curr_center_x <= sum_x / (pixel_count);
                curr_center_y <= sum_y / (pixel_count);
            end else begin
                curr_center_x <= curr_center_x;
                curr_center_y <= curr_center_y;
            end
        end
    end

    always_ff @(posedge clk, posedge reset) begin
        if (reset) begin
            center_x <= 10'd0;
            center_y <= 10'd0;
        end else begin
            center_x <= curr_center_x;
            center_y <= curr_center_y;
        end
    end

    logic [9:0] box_left, box_right, box_top, box_bottom;
    logic box_h_border, box_v_border;

    assign box_left = (center_x > (box_width/2)) ? (center_x - (box_width/2)) : 10'd0;
    assign box_right = (center_x + (box_width/2) < 10'd320) ? (center_x + (box_width/2)) : 10'd319;
    assign box_top = (center_y > (box_height/2)) ? (center_y - (box_height/2)) : 10'd0;
    assign box_bottom = (center_y + (box_height/2) < 10'd240) ? (center_y + (box_height/2)) : 10'd239;

    assign box_h_border = ((y_pixel == box_top || y_pixel == box_bottom) && 
                          (x_pixel >= box_left && x_pixel <= box_right));

    assign box_v_border = ((x_pixel == box_left || x_pixel == box_right) && 
                          (y_pixel >= box_top && y_pixel <= box_bottom));

    assign box_active = display_enable && motion_valid && (box_h_border || box_v_border);

endmodule
