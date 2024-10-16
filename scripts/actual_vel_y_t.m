function actual_vel_y_t(log_folder)
    % Dosya yolunu belirleme (log klasöründen okuma)
    file_path = fullfile(log_folder, 'states.txt');

    % Veriyi okuma (readtable kullanarak daha esnek şekilde okuma)
    data = readtable(file_path, 'Delimiter', ',');

    % Timestamp (1. kolon) ve a_vel_y (6. kolon) verilerini ayırma
    timestamp = data{:, 1};  % 1. kolon: timestamp
    a_vel_y = data{:, 6};      % 6. kolon: a_vel_y

    % Eğer veriler doğruysa, grafiği çizelim
    if ~isempty(timestamp) && ~isempty(a_vel_y)
        figure;
        plot(timestamp, a_vel_y, 'LineWidth', 1.5);
        xlabel('Timestamp (s)');
        ylabel('a_vel_y');
        title('a_vel_y vs. Timestamp');
        grid on;
    else
        disp('Veriler boş olabilir, lütfen dosya formatını kontrol edin.');
    end
end
