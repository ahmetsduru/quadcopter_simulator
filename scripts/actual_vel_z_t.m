function actual_vel_z_t(log_folder)
    % Dosya yolunu belirleme (log klasöründen okuma)
    file_path = fullfile(log_folder, 'states.txt');

    % Veriyi okuma (readtable kullanarak daha esnek şekilde okuma)
    data = readtable(file_path, 'Delimiter', ',');

    % Timestamp (1. kolon) ve a_vel_z (7. kolon) verilerini ayırma
    timestamp = data{:, 1};  % 1. kolon: timestamp
    a_vel_z = data{:, 7};      % 7. kolon: a_vel_z

    % Eğer veriler doğruysa, grafiği çizelim
    if ~isempty(timestamp) && ~isempty(a_vel_z)
        figure;
        plot(timestamp, a_vel_z, 'LineWidth', 1.5);
        xlabel('Timestamp (s)');
        ylabel('a_vel_z');
        title('a_vel_z vs. Timestamp');
        grid on;
    else
        disp('Veriler boş olabilir, lütfen dosya formatını kontrol edin.');
    end
end
