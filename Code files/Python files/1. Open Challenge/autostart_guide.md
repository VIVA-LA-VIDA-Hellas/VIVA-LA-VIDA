- Execute sudo nano /etc/systemd/system/vivalavida.service to create the vivalavida.service in systemd.

- Use this data in the service:
  [Unit]
  Description=Viva La Vida Robot Autostart
  After=network.target
  [Service]
  ExecStart=/usr/bin/python3 /home/pi/Desktop/Viva_La_Vida/1st_mission.py
  WorkingDirectory=/home/pi/Desktop/Viva_La_Vida
  StandardOutput=inherit
  StandardError=inherit
  Restart=always
  User=pi
  [Install]
  WantedBy=multi-user.target

sudo systemctl enable vivalavida.service
sudo systemctl start vivalavida.service
journalctl -u vivalavida.service -f

- Create desktop shortcuts
  Script	Purpose	Command Used
  enable_vivalavida.sh	Reloads, enables, and starts the service	sudo systemctl daemon-reload && sudo systemctl enable vivalavida.service && sudo systemctl start vivalavida.service
  disable_vivalavida.sh	Stops and disables the service	sudo systemctl stop vivalavida.service && sudo systemctl disable vivalavida.service
  status_vivalavida.sh	Shows current status and last 20 logs	sudo systemctl status vivalavida.service --no-pager && sudo journalctl -u vivalavida.service -n 20 --no-pager
  live_logs_vivalavida.sh	Streams live logs in real time	sudo journalctl -u vivalavida.service -f

