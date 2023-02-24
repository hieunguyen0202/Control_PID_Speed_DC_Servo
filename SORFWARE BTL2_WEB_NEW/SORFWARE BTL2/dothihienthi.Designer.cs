
namespace SORFWARE_BTL2
{
    partial class dothihienthi
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.components = new System.ComponentModel.Container();
            this.groupBox1 = new System.Windows.Forms.GroupBox();
            this.zedGraphPosRef = new ZedGraph.ZedGraphControl();
            this.groupBox4 = new System.Windows.Forms.GroupBox();
            this.label10 = new System.Windows.Forms.Label();
            this.button4 = new System.Windows.Forms.Button();
            this.button3 = new System.Windows.Forms.Button();
            this.btnSendSeting = new System.Windows.Forms.Button();
            this.button1 = new System.Windows.Forms.Button();
            this.label9 = new System.Windows.Forms.Label();
            this.label7 = new System.Windows.Forms.Label();
            this.label5 = new System.Windows.Forms.Label();
            this.label8 = new System.Windows.Forms.Label();
            this.label6 = new System.Windows.Forms.Label();
            this.label4 = new System.Windows.Forms.Label();
            this.txbAccMax = new System.Windows.Forms.TextBox();
            this.txbVelMax = new System.Windows.Forms.TextBox();
            this.txbPosMax = new System.Windows.Forms.TextBox();
            this.groupBox2 = new System.Windows.Forms.GroupBox();
            this.zedGraphVelocity = new ZedGraph.ZedGraphControl();
            this.groupBox3 = new System.Windows.Forms.GroupBox();
            this.zedGraphAcc = new ZedGraph.ZedGraphControl();
            this.label1 = new System.Windows.Forms.Label();
            this.groupBox1.SuspendLayout();
            this.groupBox4.SuspendLayout();
            this.groupBox2.SuspendLayout();
            this.groupBox3.SuspendLayout();
            this.SuspendLayout();
            // 
            // groupBox1
            // 
            this.groupBox1.Controls.Add(this.zedGraphPosRef);
            this.groupBox1.Location = new System.Drawing.Point(618, 78);
            this.groupBox1.Name = "groupBox1";
            this.groupBox1.Size = new System.Drawing.Size(569, 351);
            this.groupBox1.TabIndex = 0;
            this.groupBox1.TabStop = false;
            this.groupBox1.Text = "GRAPH POSITION";
            // 
            // zedGraphPosRef
            // 
            this.zedGraphPosRef.Location = new System.Drawing.Point(7, 36);
            this.zedGraphPosRef.Margin = new System.Windows.Forms.Padding(4, 4, 4, 4);
            this.zedGraphPosRef.Name = "zedGraphPosRef";
            this.zedGraphPosRef.ScrollGrace = 0D;
            this.zedGraphPosRef.ScrollMaxX = 0D;
            this.zedGraphPosRef.ScrollMaxY = 0D;
            this.zedGraphPosRef.ScrollMaxY2 = 0D;
            this.zedGraphPosRef.ScrollMinX = 0D;
            this.zedGraphPosRef.ScrollMinY = 0D;
            this.zedGraphPosRef.ScrollMinY2 = 0D;
            this.zedGraphPosRef.Size = new System.Drawing.Size(546, 286);
            this.zedGraphPosRef.TabIndex = 0;
            this.zedGraphPosRef.UseExtendedPrintDialog = true;
            // 
            // groupBox4
            // 
            this.groupBox4.Controls.Add(this.label10);
            this.groupBox4.Controls.Add(this.button4);
            this.groupBox4.Controls.Add(this.button3);
            this.groupBox4.Controls.Add(this.btnSendSeting);
            this.groupBox4.Controls.Add(this.button1);
            this.groupBox4.Controls.Add(this.label9);
            this.groupBox4.Controls.Add(this.label7);
            this.groupBox4.Controls.Add(this.label5);
            this.groupBox4.Controls.Add(this.label8);
            this.groupBox4.Controls.Add(this.label6);
            this.groupBox4.Controls.Add(this.label4);
            this.groupBox4.Controls.Add(this.txbAccMax);
            this.groupBox4.Controls.Add(this.txbVelMax);
            this.groupBox4.Controls.Add(this.txbPosMax);
            this.groupBox4.Location = new System.Drawing.Point(120, 133);
            this.groupBox4.Name = "groupBox4";
            this.groupBox4.Size = new System.Drawing.Size(420, 267);
            this.groupBox4.TabIndex = 8;
            this.groupBox4.TabStop = false;
            this.groupBox4.Text = "Operation";
            // 
            // label10
            // 
            this.label10.Location = new System.Drawing.Point(6, 199);
            this.label10.Name = "label10";
            this.label10.Size = new System.Drawing.Size(165, 38);
            this.label10.TabIndex = 3;
            this.label10.Text = "label10";
            // 
            // button4
            // 
            this.button4.Location = new System.Drawing.Point(276, 199);
            this.button4.Name = "button4";
            this.button4.Size = new System.Drawing.Size(127, 42);
            this.button4.TabIndex = 2;
            this.button4.Text = "GET";
            this.button4.UseVisualStyleBackColor = true;
            // 
            // button3
            // 
            this.button3.Location = new System.Drawing.Point(276, 151);
            this.button3.Name = "button3";
            this.button3.Size = new System.Drawing.Size(127, 42);
            this.button3.TabIndex = 2;
            this.button3.Text = "RUN";
            this.button3.UseVisualStyleBackColor = true;
            // 
            // btnSendSeting
            // 
            this.btnSendSeting.Location = new System.Drawing.Point(276, 103);
            this.btnSendSeting.Name = "btnSendSeting";
            this.btnSendSeting.Size = new System.Drawing.Size(127, 42);
            this.btnSendSeting.TabIndex = 2;
            this.btnSendSeting.Text = "SEND SETING";
            this.btnSendSeting.UseVisualStyleBackColor = true;
            this.btnSendSeting.Click += new System.EventHandler(this.btnSendSeting_Click);
            // 
            // button1
            // 
            this.button1.Location = new System.Drawing.Point(276, 54);
            this.button1.Name = "button1";
            this.button1.Size = new System.Drawing.Size(127, 42);
            this.button1.TabIndex = 2;
            this.button1.Text = "MOTION";
            this.button1.UseVisualStyleBackColor = true;
            this.button1.Click += new System.EventHandler(this.button1_Click);
            // 
            // label9
            // 
            this.label9.AutoSize = true;
            this.label9.Location = new System.Drawing.Point(176, 149);
            this.label9.Name = "label9";
            this.label9.Size = new System.Drawing.Size(76, 17);
            this.label9.TabIndex = 1;
            this.label9.Text = "Unit/Sec^2";
            // 
            // label7
            // 
            this.label7.AutoSize = true;
            this.label7.Location = new System.Drawing.Point(176, 103);
            this.label7.Name = "label7";
            this.label7.Size = new System.Drawing.Size(61, 17);
            this.label7.TabIndex = 1;
            this.label7.Text = "Unit/Sec";
            // 
            // label5
            // 
            this.label5.AutoSize = true;
            this.label5.Location = new System.Drawing.Point(176, 56);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(33, 17);
            this.label5.TabIndex = 1;
            this.label5.Text = "Unit";
            // 
            // label8
            // 
            this.label8.AutoSize = true;
            this.label8.Location = new System.Drawing.Point(6, 149);
            this.label8.Name = "label8";
            this.label8.Size = new System.Drawing.Size(56, 17);
            this.label8.TabIndex = 1;
            this.label8.Text = "AccMax";
            // 
            // label6
            // 
            this.label6.AutoSize = true;
            this.label6.Location = new System.Drawing.Point(6, 103);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(53, 17);
            this.label6.TabIndex = 1;
            this.label6.Text = "VelMax";
            // 
            // label4
            // 
            this.label4.AutoSize = true;
            this.label4.Location = new System.Drawing.Point(6, 56);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(54, 17);
            this.label4.TabIndex = 1;
            this.label4.Text = "PosRef";
            // 
            // txbAccMax
            // 
            this.txbAccMax.Location = new System.Drawing.Point(70, 148);
            this.txbAccMax.Name = "txbAccMax";
            this.txbAccMax.Size = new System.Drawing.Size(100, 22);
            this.txbAccMax.TabIndex = 0;
            // 
            // txbVelMax
            // 
            this.txbVelMax.Location = new System.Drawing.Point(70, 102);
            this.txbVelMax.Name = "txbVelMax";
            this.txbVelMax.Size = new System.Drawing.Size(100, 22);
            this.txbVelMax.TabIndex = 0;
            // 
            // txbPosMax
            // 
            this.txbPosMax.Location = new System.Drawing.Point(70, 55);
            this.txbPosMax.Name = "txbPosMax";
            this.txbPosMax.Size = new System.Drawing.Size(100, 22);
            this.txbPosMax.TabIndex = 0;
            // 
            // groupBox2
            // 
            this.groupBox2.Controls.Add(this.zedGraphVelocity);
            this.groupBox2.Location = new System.Drawing.Point(18, 460);
            this.groupBox2.Name = "groupBox2";
            this.groupBox2.Size = new System.Drawing.Size(569, 351);
            this.groupBox2.TabIndex = 0;
            this.groupBox2.TabStop = false;
            this.groupBox2.Text = "GRAPH VAN TOC";
            // 
            // zedGraphVelocity
            // 
            this.zedGraphVelocity.Location = new System.Drawing.Point(7, 36);
            this.zedGraphVelocity.Margin = new System.Windows.Forms.Padding(4, 4, 4, 4);
            this.zedGraphVelocity.Name = "zedGraphVelocity";
            this.zedGraphVelocity.ScrollGrace = 0D;
            this.zedGraphVelocity.ScrollMaxX = 0D;
            this.zedGraphVelocity.ScrollMaxY = 0D;
            this.zedGraphVelocity.ScrollMaxY2 = 0D;
            this.zedGraphVelocity.ScrollMinX = 0D;
            this.zedGraphVelocity.ScrollMinY = 0D;
            this.zedGraphVelocity.ScrollMinY2 = 0D;
            this.zedGraphVelocity.Size = new System.Drawing.Size(546, 286);
            this.zedGraphVelocity.TabIndex = 0;
            this.zedGraphVelocity.UseExtendedPrintDialog = true;
            // 
            // groupBox3
            // 
            this.groupBox3.Controls.Add(this.zedGraphAcc);
            this.groupBox3.Location = new System.Drawing.Point(618, 460);
            this.groupBox3.Name = "groupBox3";
            this.groupBox3.Size = new System.Drawing.Size(569, 351);
            this.groupBox3.TabIndex = 0;
            this.groupBox3.TabStop = false;
            this.groupBox3.Text = "GRAPH GIA TOC";
            // 
            // zedGraphAcc
            // 
            this.zedGraphAcc.Location = new System.Drawing.Point(7, 36);
            this.zedGraphAcc.Margin = new System.Windows.Forms.Padding(4, 4, 4, 4);
            this.zedGraphAcc.Name = "zedGraphAcc";
            this.zedGraphAcc.ScrollGrace = 0D;
            this.zedGraphAcc.ScrollMaxX = 0D;
            this.zedGraphAcc.ScrollMaxY = 0D;
            this.zedGraphAcc.ScrollMaxY2 = 0D;
            this.zedGraphAcc.ScrollMinX = 0D;
            this.zedGraphAcc.ScrollMinY = 0D;
            this.zedGraphAcc.ScrollMinY2 = 0D;
            this.zedGraphAcc.Size = new System.Drawing.Size(546, 286);
            this.zedGraphAcc.TabIndex = 0;
            this.zedGraphAcc.UseExtendedPrintDialog = true;
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Font = new System.Drawing.Font("Microsoft Sans Serif", 25.8F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label1.ForeColor = System.Drawing.SystemColors.Highlight;
            this.label1.Location = new System.Drawing.Point(434, 9);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(278, 51);
            this.label1.TabIndex = 9;
            this.label1.Text = "CÁC ĐỒ THỊ";
            // 
            // dothihienthi
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(8F, 16F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(1282, 823);
            this.Controls.Add(this.label1);
            this.Controls.Add(this.groupBox4);
            this.Controls.Add(this.groupBox3);
            this.Controls.Add(this.groupBox2);
            this.Controls.Add(this.groupBox1);
            this.Name = "dothihienthi";
            this.Text = "dothihienthi";
            this.Load += new System.EventHandler(this.dothihienthi_Load);
            this.groupBox1.ResumeLayout(false);
            this.groupBox4.ResumeLayout(false);
            this.groupBox4.PerformLayout();
            this.groupBox2.ResumeLayout(false);
            this.groupBox3.ResumeLayout(false);
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.GroupBox groupBox1;
        private System.Windows.Forms.GroupBox groupBox4;
        private System.Windows.Forms.Label label10;
        private System.Windows.Forms.Button button4;
        private System.Windows.Forms.Button button3;
        private System.Windows.Forms.Button btnSendSeting;
        private System.Windows.Forms.Button button1;
        private System.Windows.Forms.Label label9;
        private System.Windows.Forms.Label label7;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.Label label8;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.TextBox txbAccMax;
        private System.Windows.Forms.TextBox txbVelMax;
        private System.Windows.Forms.TextBox txbPosMax;
        private ZedGraph.ZedGraphControl zedGraphPosRef;
        private System.Windows.Forms.GroupBox groupBox2;
        private ZedGraph.ZedGraphControl zedGraphVelocity;
        private System.Windows.Forms.GroupBox groupBox3;
        private ZedGraph.ZedGraphControl zedGraphAcc;
        private System.Windows.Forms.Label label1;
    }
}