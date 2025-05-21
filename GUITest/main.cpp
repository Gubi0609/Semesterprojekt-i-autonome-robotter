// main.cpp
#include <wx/wx.h>
#include <wx/grid.h>
#include <wx/timer.h>
#include <wx/process.h>
#include <wx/txtstrm.h>  // For wxTextInputStream and wxTextOutputStream
#include <wx/wfstream.h> // Optional if you use file streams, but not needed here


class SettingsDialog : public wxDialog {
public:
    SettingsDialog(wxWindow* parent)
        : wxDialog(parent, wxID_ANY, "Game Settings", wxDefaultPosition, wxSize(300, 350)) {

        // Main sizer for the dialog
        wxBoxSizer* mainSizer = new wxBoxSizer(wxVERTICAL);

        // Color choice
        wxStaticBoxSizer* colorSizer = new wxStaticBoxSizer(wxVERTICAL, this, "Choose color");
        wxArrayString colors;
        colors.Add("White");
        colors.Add("Black");
        colorChoice = new wxRadioBox(this, wxID_ANY, "", wxDefaultPosition, wxDefaultSize, colors, 2, wxRA_SPECIFY_COLS);
        colorSizer->Add(colorChoice, 0, wxALL, 5);
        mainSizer->Add(colorSizer, 0, wxALL | wxEXPAND, 5);

        // Difficulty choice
        wxStaticBoxSizer* difficultySizer = new wxStaticBoxSizer(wxVERTICAL, this, "Choose difficulty");
        wxArrayString difficulties;
        difficulties.Add("1");
        difficulties.Add("2");
        difficulties.Add("3");
        difficulties.Add("4");
        difficulties.Add("5");
        difficultyChoice = new wxRadioBox(this, wxID_ANY, "", wxDefaultPosition, wxDefaultSize, difficulties, 5, wxRA_SPECIFY_COLS);
        difficultySizer->Add(difficultyChoice, 0, wxALL, 5);
        mainSizer->Add(difficultySizer, 0, wxALL | wxEXPAND, 5);

        // Time choice
        wxStaticBoxSizer* timeSizer = new wxStaticBoxSizer(wxVERTICAL, this, "Choose time");
        wxArrayString times;
        times.Add("5 min");
        times.Add("10 min");
        times.Add("30 min");
        timeChoice = new wxRadioBox(this, wxID_ANY, "", wxDefaultPosition, wxDefaultSize, times, 3, wxRA_SPECIFY_COLS);
        timeSizer->Add(timeChoice, 0, wxALL, 5);
        mainSizer->Add(timeSizer, 0, wxALL | wxEXPAND, 5);

        // OK button
        wxButton* okButton = new wxButton(this, wxID_OK, "OK", wxDefaultPosition, wxDefaultSize);
        mainSizer->Add(okButton, 0, wxALIGN_CENTER | wxALL, 5);

        SetSizerAndFit(mainSizer);
    }

    wxString GetColor() { return colorChoice->GetStringSelection(); }
    wxString GetDifficulty() { return difficultyChoice->GetStringSelection(); }
    wxString GetTime() { return timeChoice->GetStringSelection(); }

private:
    wxRadioBox* colorChoice;
    wxRadioBox* difficultyChoice;
    wxRadioBox* timeChoice;
};


class ChessFrame : public wxFrame {
public:
    ChessFrame(const wxString& time) : wxFrame(nullptr, wxID_ANY, "UR5 Chess GUI", wxDefaultPosition, wxSize(600, 400)) {
        // // Start robot program as child process
        // robotProcess = new wxProcess(this);
        // robotProcess->Redirect(); // Enable reading and writing
        // robotPID = wxExecute("/path/to/your/robot_program", wxEXEC_ASYNC, robotProcess);
        // if (robotPID == 0) {
        //     wxMessageBox("Failed to launch robot process", "Error", wxICON_ERROR);
        // }


        // Bind the event for when output is available
        Bind(wxEVT_IDLE, &ChessFrame::OnIdle, this); // Poll during idle time
        
        wxPanel* panel = new wxPanel(this);
        wxBoxSizer* mainSizer = new wxBoxSizer(wxHORIZONTAL);

        // Left side sizer for Registered Move, Robot Move, and Timer
        wxBoxSizer* leftSizer = new wxBoxSizer(wxVERTICAL);

        // Create a horizontal box sizer for "Registered Move" label and text
        wxBoxSizer* registeredMoveSizer = new wxBoxSizer(wxHORIZONTAL);
        wxStaticText* registeredMoveLabel = new wxStaticText(panel, wxID_ANY, "Registered Move:");
        registeredMoveText = new wxStaticText(panel, wxID_ANY, "Placeholder text", wxDefaultPosition, wxSize(200, -1));
        registeredMoveSizer->Add(registeredMoveLabel, 0, wxALIGN_CENTER_VERTICAL | wxALL, 5);  // Align vertically
        registeredMoveSizer->Add(registeredMoveText, 1, wxALIGN_CENTER_VERTICAL | wxALL, 5);   // Allow stretching

        // Create a horizontal box sizer for "Robot Move" label and text
        wxBoxSizer* robotMoveSizer = new wxBoxSizer(wxHORIZONTAL);
        wxStaticText* robotMoveLabel = new wxStaticText(panel, wxID_ANY, "Robot Move:");
        robotMoveText = new wxStaticText(panel, wxID_ANY, "Placeholder text", wxDefaultPosition, wxSize(200, -1));
        robotMoveSizer->Add(robotMoveLabel, 0, wxALIGN_CENTER_VERTICAL | wxALL, 5);  // Align vertically
        robotMoveSizer->Add(robotMoveText, 1, wxALIGN_CENTER_VERTICAL | wxALL, 5);   // Allow stretching

        // Add the sizers to the leftSizer
        leftSizer->Add(registeredMoveSizer, 0, wxALL | wxEXPAND, 5);
        leftSizer->Add(robotMoveSizer, 0, wxALL | wxEXPAND, 5);

        // Horizontal sizer for the two timers
        wxBoxSizer* timerRow = new wxBoxSizer(wxHORIZONTAL);

        // Timer 1 (Player)
        wxStaticText* timer1Label = new wxStaticText(panel, wxID_ANY, "Player Timer:");
        timer1Text = new wxStaticText(panel, wxID_ANY, time);
        timerRow->Add(timer1Label, 0, wxRIGHT, 5);
        timerRow->Add(timer1Text, 0, wxRIGHT, 15);

        // Timer 2 (Robot)
        wxStaticText* timer2Label = new wxStaticText(panel, wxID_ANY, "Robot Timer:");
        timer2Text = new wxStaticText(panel, wxID_ANY, time);
        timerRow->Add(timer2Label, 0, wxRIGHT, 5);
        timerRow->Add(timer2Text, 0);

        leftSizer->Add(timerRow, 0, wxALL, 5);

        // Parse time string
        long minutes;
        time.BeforeFirst(' ').ToLong(&minutes);
        playerSecondsLeft = robotSecondsLeft = minutes * 60;

        // Timer logic
        timer = new wxTimer(this, wxID_ANY);
        timer->Start(1000);
        Bind(wxEVT_TIMER, &ChessFrame::OnTimer, this, timer->GetId());

        // Move List on the right side
        wxBoxSizer* rightSizer = new wxBoxSizer(wxVERTICAL);
        wxStaticText* moveListLabel = new wxStaticText(panel, wxID_ANY, "Move List:");
        
        // Create the wxGrid for the Move List
        moveListGrid = new wxGrid(panel, wxID_ANY, wxDefaultPosition, wxSize(400, 300));

        // Set number of columns: 3 (Move number, Player Move, Robot Move)
        moveListGrid->CreateGrid(0, 3); // Start with 0 rows, 3 columns
        moveListGrid->SetColLabelValue(0, "Move #");
        moveListGrid->SetColLabelValue(1, "Player Move");
        moveListGrid->SetColLabelValue(2, "Robot Move");

        // Set column sizes for better readability
        moveListGrid->SetColSize(0, 50); // Move number column width
        moveListGrid->SetColSize(1, 150); // Player Move column width
        moveListGrid->SetColSize(2, 150); // Robot Move column width

        // Make the grid editable for player moves if you wish
        moveListGrid->EnableEditing(true);
        moveListGrid->SetSelectionMode(wxGrid::wxGridSelectRows);

        // Add the grid to the rightSizer
        rightSizer->Add(moveListLabel, 0, wxALL, 5);
        rightSizer->Add(moveListGrid, 1, wxALL | wxEXPAND, 5);


        // Add left and right sizers to the main sizer
        mainSizer->Add(leftSizer, 0, wxALL, 5);
        mainSizer->Add(rightSizer, 1, wxALL | wxEXPAND, 5);

        panel->SetSizer(mainSizer);
        mainSizer->Fit(this);  // Resize the frame to fit all contents
        mainSizer->SetSizeHints(this); // Optional: sets min size to avoid shrinking below fit size

    }

    void SendPlayerColorToRobot(const wxString& color) {
        if (!robotProcess) return;

        wxOutputStream* in = robotProcess->GetOutputStream();
        if (in && in->IsOk()) {
            wxTextOutputStream text(*in);
            text.WriteString(color + "\n");  // Send player color
            text.Flush();
        }
    }

private:
    wxGrid* moveListGrid;
    wxStaticText* registeredMoveText;
    wxStaticText* robotMoveText;

    wxStaticText* timer1Text;
    wxStaticText* timer2Text;
    wxTimer* timer;

    int playerSecondsLeft;
    int robotSecondsLeft;
    bool isPlayerTurn = true; // Assume player starts


    wxProcess* robotProcess = nullptr;
    long robotPID = 0;

    void UpdateMoveList(const wxString& playerMove, const wxString& robotMove) {
        int row = moveListGrid->GetNumberRows();
        moveListGrid->AppendRows(1);  // Add a new row at the end
        moveListGrid->SetCellValue(row, 0, wxString::Format("%d", row + 1));  // Move number (1, 2, 3, ...)
        moveListGrid->SetCellValue(row, 1, playerMove);  // Player move
        moveListGrid->SetCellValue(row, 2, robotMove);   // Robot move
    }


    void OnIdle(wxIdleEvent& event) {
        if (!robotProcess) return;

        wxInputStream* out = robotProcess->GetInputStream();
        if (out && out->CanRead()) {
            wxTextInputStream text(*out);
            while (out->CanRead()) {
                wxString line = text.ReadLine();

                // Parse robot output and update GUI
                if (line.StartsWith("Registered Move:")) {
                    registeredMoveText->SetLabel(line.AfterFirst(':').Trim());
                    isPlayerTurn = false;
                } else if (line.StartsWith("Robot Move:")) {
                    robotMoveText->SetLabel(line.AfterFirst(':').Trim());
                    // Now update the grid with both moves
                    UpdateMoveList(registeredMoveText->GetLabel(), robotMoveText->GetLabel());
                    isPlayerTurn = true;
                }
            }
        }

        // Keep GUI responsive
        event.RequestMore();
    }

    void OnTimer(wxTimerEvent& event) {
        if (isPlayerTurn) {
            if (playerSecondsLeft > 0) playerSecondsLeft--;
        } else {
            if (robotSecondsLeft > 0) robotSecondsLeft--;
        }


        int pMin = playerSecondsLeft / 60;
        int pSec = playerSecondsLeft % 60;
        timer1Text->SetLabel(wxString::Format("%02d:%02d", pMin, pSec));

        int rMin = robotSecondsLeft / 60;
        int rSec = robotSecondsLeft % 60;
        timer2Text->SetLabel(wxString::Format("%02d:%02d", rMin, rSec));
    }

};


class ChessApp : public wxApp {
public:
    virtual bool OnInit() override {
        // Step 1: Show the Settings Dialog on startup
        SettingsDialog* settingsDialog = new SettingsDialog(nullptr);
        wxString time;
        wxString color;
        wxString difficulty;
        if (settingsDialog->ShowModal() == wxID_OK) {
            // Step 2: Get the user settings after they press OK
            color = settingsDialog->GetColor();
            difficulty = settingsDialog->GetDifficulty();
            time = settingsDialog->GetTime();

            // You can use these settings for game initialization here
            wxLogMessage("Color: %s, Difficulty: %s, Time: %s", color, difficulty, time);
        }
        settingsDialog->Destroy();  // Clean up the settings dialog

        // Step 3: Create and show the main frame (your chess game window)
        ChessFrame* frame = new ChessFrame(time);
        frame->Show(true);  // Show the main frame window

        frame->SendPlayerColorToRobot(color == "White" ? "w" : "b");

        return true;  // Return true to indicate that initialization was successful
    }
};

wxIMPLEMENT_APP(ChessApp);
