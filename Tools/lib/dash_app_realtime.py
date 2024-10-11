import json
import socket
import dash
from dash import dcc, html
from dash.dependencies import Input, Output, State
import plotly.graph_objs as go
import threading
import csv
from datetime import datetime
import time
import warnings
import pandas as pd
import plotly.io as pio
import webbrowser  # Import webbrowser module to open HTML file automatically
import base64
import io

# Global variables to store dynamic data received from the robot server
robot_data = {}
socket_active = False
client_thread = None
csv_file = None
csv_writer = None
stop_requested = False
isReceiving = False
bad_data_rev_count = 0
csv_filename = None

# Configure the server IP address and port
HOST = '192.168.43.1'#'192.168.43.1' '127.0.0.1'  # Replace with the robot's IP address
PORT = 12345
BUFFER_SIZE = 8192

# Function to save received JSON data to a CSV file in real-time
def save_data_to_csv(json_data):
    global csv_writer, csv_file

    # If this is the first time data is written, write the header dynamically
    if csv_writer.fieldnames is None:
        csv_writer.fieldnames = json_data.keys()
        csv_writer.writeheader()

    # Write the current JSON data as a row
    csv_writer.writerow(json_data)

    # Flush the data to ensure it's written to the file immediately
    csv_file.flush()

# Function to connect to the robot server and receive data, throwing exceptions
def connect_to_robot():
    global robot_data, socket_active, csv_writer, csv_file, isReceiving, csv_filename
    global bad_data_rev_count

    bad_data_rev_count = 0
    robot_data['nt_error'] = []
    buffer = ""  # Buffer to hold partial JSON data

    try:
        # Set up a client socket to connect to the robot server
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            # warnings.filterwarnings('ignore',category=DeprecationWarning)
            s.connect((HOST, PORT))  # This will throw an exception if it fails
            print(f"Connected to robot server at {HOST}:{PORT}")
            
            # Create a unique CSV file name based on the current date and time
            current_time = datetime.now().strftime("%Y%m%d_%H%M%S")
            csv_filename = f"{current_time}.csv"
    
            # Open the CSV file in append mode to save real-time data
            csv_file = open(csv_filename, mode='a', newline='')
            csv_writer = csv.DictWriter(csv_file, fieldnames=None)
    
            while socket_active:
                isReceiving = True
                # Receive data from the server
                data = s.recv(BUFFER_SIZE).decode('utf-8')
    
                # Append received data to the buffer
                buffer += data
    
                # Split the buffer by newlines to get complete JSON objects
                json_strings = buffer.split('\n')
    
                # Keep the last (potentially incomplete) JSON object in the buffer
                buffer = json_strings.pop()
    
                # Process each complete JSON string
                for json_string in json_strings:
                    if json_string.strip():  # Ignore empty strings
                        try:
                            json_data = json.loads(json_string)
    
                            # Save data to CSV in real-time
                            save_data_to_csv(json_data)
    
                            # Update robot_data dictionary with new keys and values
                            for key, value in json_data.items():
                                if key not in robot_data:
                                    robot_data[key] = []  # Initialize new key with an empty list
                                robot_data[key].append(value)
                            robot_data['nt_error'].append(bad_data_rev_count)
    
                            if len(robot_data['time']) > 50:
                                for key in robot_data:
                                    (robot_data[key]).pop(0)
									
                            # Print the received data for debugging
                            # print(f"Received data: {json_data}")
                        except json.JSONDecodeError as e:
                            bad_data_rev_count = (bad_data_rev_count+1)%255
                            # Reraise the JSON decoding error to allow higher-level handling
                            #raise e
                            print(f"An JSON error occurred: {e}")
    except ConnectionRefusedError as e:
        print(f"Connection to the robot server at {HOST}:{PORT} failed.")
        isReceiving = False
        raise e
    except Exception as e:
        print(f"An error occurred: {e}")
        isReceiving = False
        raise e

# Function to send user input to the robot server in JSON format
def monitor_user_input(socket):
    previous_input = {"sampleValue1": None, "sampleValue2": None}

    while True:
        # Get user input
        try:
            sample_value_1 = input("Enter sample value 1: ")
            sample_value_2 = input("Enter sample value 2: ")
    
            # Create a JSON object with the input values
            input_data = {
                "sampleValue1": sample_value_1,
                "sampleValue2": sample_value_2
            }
        except Exception as e:
            pass

        # Check if the input values have changed
        if input_data != previous_input:
            # Send the JSON data to the server only if input has changed
            socket.sendall((json.dumps(input_data) + "\n").encode('utf-8'))
            print(f"Sent user input: {input_data}")

            # Update the previous input to the current input
            previous_input = input_data
        else:
            print("No changes detected, not sending data.")
# Function to start the connection and CSV logging, with a retry mechanism
def start_receiving_data():
    global socket_active, client_thread, csv_file, csv_writer, isReceiving

    if not socket_active:
        socket_active = True
        isReceiving = False

        # Retry mechanism to wait until the server is available
        while socket_active:
            try:
                while not isReceiving:
                    print(f"Attempting to connect to robot server at {HOST}:{PORT}...")
                    # Start the thread to connect to the robot and receive data

                    client_thread = threading.Thread(target=connect_to_robot)
                    client_thread.start()
                    time.sleep(3)
                # client_thread.join()  # Wait for the thread to finish
                if isReceiving:
                    break  # Exit the loop once a connection is successfully established
                # else:
            except Exception as e:
                # Handle any exceptions thrown by connect_to_robot()
                print(f"Error: {e}. Retrying in 1 second...")
                time.sleep(1)  # Wait before retrying
                

# Function to stop receiving data and close the CSV file
def stop_receiving_data():
    global socket_active, csv_file, stop_requested

    stop_requested = True
    if socket_active:
        socket_active = False
        if client_thread is not None:
            client_thread.join()  # Wait for the thread to finish

        # Close the CSV file
        if csv_file:
            csv_file.close()

        print("Disconnected from robot server and CSV file closed.")

# Function to generate the HTML file from CSV data
def generate_html_from_csv(filename):   

    if filename is not None:
        # Read the CSV file into a DataFrame
        df = pd.read_csv(filename)

        # Create a Plotly figure
        fig = go.Figure()

        # Plot each column from the CSV
        for column in df.columns:
            if pd.api.types.is_numeric_dtype(df[column]):  # Only plot numeric columns
                fig.add_trace(go.Scatter(x=df.index, y=df[column], mode='lines+markers', name=column))

        # Customize the layout
        fig.update_layout(
            title='Data from '+filename,
            xaxis_title='Time',
            yaxis_title='Values'
        )
        fig.update_traces(visible='legendonly')

        # Generate the HTML file
        html_filename = filename.replace('.csv', '.html')
        pio.write_html(fig, file=html_filename, auto_open=True)
        print(f"HTML file generated: {html_filename}")
        
        # Automatically open the generated HTML file in the default web browser
        # webbrowser.open(f'file://{html_filename}', new=2)  # Open in a new tab, if possible
    else:
        print("No CSV file available to generate HTML.")

# Function to handle the uploaded CSV file and generate the HTML file
def handle_uploaded_csv(contents, filename):
    global uploaded_csv_filename

    # Decode the uploaded CSV file content
    content_type, content_string = contents.split(',')
    decoded = base64.b64decode(content_string)
    uploaded_csv_filename = filename

    # Save the uploaded file to a temporary location
    with open(uploaded_csv_filename, 'wb') as f:
        f.write(decoded)

    # Generate HTML from the uploaded CSV
    generate_html_from_csv(uploaded_csv_filename)


# Create a Dash application for real-time data visualization
app = dash.Dash(__name__)

# Define the layout of the Dash app with buttons for controlling data reception
app.layout = html.Div([
    html.H1('Robot Telemetry'),

    # Start and Stop buttons
    html.Button('Start Socket and Receive Data', id='start-button', n_clicks=0),
    html.Button('Stop Receiving Data and Disconnect Server', id='stop-button', n_clicks=0, style={'margin-left': '20px'}),

    dcc.Graph(id='live-graph'),
    dcc.Interval(id='interval-component', interval=100, n_intervals=0),  # Update every second

    # Button to generate HTML file
    html.Button('Generate HTML file from recording', id='generate-html-button', n_clicks=0, style={'margin-left': '20px'}),

    # File upload for user to select and load a CSV file
    dcc.Upload(
        id='upload-data',
        children=html.Div([
            'Drag and Drop or ',
            html.A('Select a CSV File')
        ]),
        style={
            'width': '100%',
            'height': '60px',
            'lineHeight': '60px',
            'borderWidth': '1px',
            'borderStyle': 'dashed',
            'borderRadius': '5px',
            'textAlign': 'center',
            'margin': '10px'
        },
        multiple=False  # Single file upload only
    ),

])

# Callback to handle button clicks and start/stop receiving data
@app.callback(
    Output('interval-component', 'disabled'),
    [Input('start-button', 'n_clicks'), Input('stop-button', 'n_clicks')]
)
def control_data_reception(start_clicks, stop_clicks):
    global socket_active

    if start_clicks > stop_clicks:
        start_clicks = stop_clicks = 0
        start_receiving_data()
        return False  # Enable the graph update interval
    elif stop_clicks > start_clicks:
        start_clicks = stop_clicks = 0
        stop_receiving_data()
        return True  # Disable the graph update interval
    return True  # Disable the interval by default

# Callback to update the graph with the received data
@app.callback(
    Output('live-graph', 'figure'),
    [Input('interval-component', 'n_intervals')]
)
def update_graph(n):
    fig = go.Figure()

    # Dynamically plot all numerical data in robot_data
    for key, values in robot_data.items():
        if key != "time" and len(values) > 0:
            if isinstance(values[0], (int, float)):  # Only plot numeric data
                fig.add_trace(go.Scatter(x=robot_data['time'], y=values, mode='lines+markers', name=key))
                

    # Customize the layout
    fig.update_layout(
        # title='Real-Time Data',
        xaxis_title='Time (Data Points)',
        yaxis_title='Values'
    )

    return fig

# Callback to generate HTML file when the button is clicked
@app.callback(
    Output('generate-html-button', 'children'),
    [Input('generate-html-button', 'n_clicks')]
)
def generate_html(n_clicks):
    global csv_filename
    # csv_filename = '20240919_191050.csv'
    if n_clicks > 0 :
        if csv_filename is not None:
            generate_html_from_csv(csv_filename)
            return "HTML File Generated"
        else:
            print("Please Run Data Collection First")
    return "Generate HTML File from Record..."

# Callback to handle file upload and generate HTML from uploaded CSV
@app.callback(
    Output('upload-data', 'children'),
    [Input('upload-data', 'contents')],
    [State('upload-data', 'filename')]
)
def handle_file_upload(contents, filename):
    if contents is not None:
        handle_uploaded_csv(contents, filename)
        return html.Div(['File Processed: ', filename])
    else:
        return html.Div(['Drag and Drop or ', html.A('Select a CSV File')])
        

# Run the Dash app
if __name__ == '__main__':
    app.run_server(debug=True)
