using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.IO.Ports;


namespace Control_Tool
{
    public partial class Form1 : Form
    {

        List<String> output = new List<string>();

        public Form1()
        {
            InitializeComponent();
        }

        private void Form1_Load(object sender, EventArgs e)
        {
            
            comboBox1.Items.AddRange(SerialPort.GetPortNames());
            comboBox1.SelectedIndex = 0;

            serialPort1.Close();

            serialPort1.PortName = (String)comboBox1.Items[comboBox1.SelectedIndex];
            
            serialPort1.Open();
                
        }

        private void comboBox1_SelectedIndexChanged(object sender, EventArgs e)
        {

            try
            {

                serialPort1.Close();

                serialPort1.PortName = (String)comboBox1.Items[comboBox1.SelectedIndex];

                serialPort1.Open();

            }
            catch (Exception)
            {
                
            }


        }

        private void serialPort1_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            
            char[] buffer = new char[500];

            int read = serialPort1.Read(buffer, 0, 500);

            Console.Write(buffer, 0, read);
            
        }

        private void comboBox2_SelectedIndexChanged(object sender, EventArgs e)
        {

            serialPort1.Close();

            serialPort1.BaudRate = Int32.Parse((String) comboBox2.Items[comboBox2.SelectedIndex]);

            serialPort1.Open();

        }

        private void Form1_FormClosing(object sender, FormClosingEventArgs e)
        {
            serialPort1.Close();
        }

        private void timer1_Tick(object sender, EventArgs e)
        {



        }

        private void checkedListBox1_SelectedIndexChanged(object sender, EventArgs e)
        {

        }
    }
}
