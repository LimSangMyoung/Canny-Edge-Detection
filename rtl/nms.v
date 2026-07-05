mmodule nms #(
    parameter DATA_WIDTH = 8
)(
    input  wire                    i_clk,
    input  wire                    i_rst,

    input  wire                    i_valid,

    input  wire [DATA_WIDTH-1:0]   i_mag_00,
    input  wire [DATA_WIDTH-1:0]   i_mag_01,
    input  wire [DATA_WIDTH-1:0]   i_mag_02,
    input  wire [DATA_WIDTH-1:0]   i_mag_10,
    input  wire [DATA_WIDTH-1:0]   i_mag_11,
    input  wire [DATA_WIDTH-1:0]   i_mag_12,
    input  wire [DATA_WIDTH-1:0]   i_mag_20,
    input  wire [DATA_WIDTH-1:0]   i_mag_21,
    input  wire [DATA_WIDTH-1:0]   i_mag_22,

    input  wire [1:0]              i_dir,

    output reg                     o_valid,
    output reg [DATA_WIDTH-1:0]    o_pixel
);

    localparam DIR_0   = 2'b00;
    localparam DIR_45  = 2'b01;
    localparam DIR_90  = 2'b10;
    localparam DIR_135 = 2'b11;

    reg [DATA_WIDTH-1:0] neighbor_a;
    reg [DATA_WIDTH-1:0] neighbor_b;

    always @(*) begin
        case (i_dir)
            DIR_0: begin
                neighbor_a = i_mag_10;
                neighbor_b = i_mag_12;
            end

            DIR_45: begin
                neighbor_a = i_mag_02;
                neighbor_b = i_mag_20;
            end

            DIR_90: begin
                neighbor_a = i_mag_01;
                neighbor_b = i_mag_21;
            end

            DIR_135: begin
                neighbor_a = i_mag_00;
                neighbor_b = i_mag_22;
            end

            default: begin
                neighbor_a = i_mag_10;
                neighbor_b = i_mag_12;
            end
        endcase
    end

    always @(posedge i_clk or posedge i_rst) begin
        if (i_rst) begin
            o_valid <= 1'b0;
            o_pixel <= {DATA_WIDTH{1'b0}};
        end
        else begin
            o_valid <= i_valid;

            if (i_valid) begin
                if ((i_mag_11 >= neighbor_a) && (i_mag_11 >= neighbor_b))
                    o_pixel <= i_mag_11;
                else
                    o_pixel <= {DATA_WIDTH{1'b0}};
            end
            else begin
                o_pixel <= {DATA_WIDTH{1'b0}};
            end
        end
    end

endmodule
