function actual_vel_x_t(log_folder)
    % Dosya yolunu belirleme (log klasöründen okuma)
    file_path = fullfile(log_folder, 'states.txt');

    % Veriyi okuma (readtable kullanarak daha esnek şekilde okuma)
    data = readtable(file_path, 'Delimiter', ',');

    % Timestamp (1. kolon) ve a_vel_x (5. kolon) verilerini ayırma
    timestamp = data{:, 1};  % 1. kolon: timestamp
    a_vel_x = data{:, 5};      % 5. kolon: a_vel_x

    % Eğer veriler doğruysa, grafiği çizelim
    if ~isempty(timestamp) && ~isempty(a_vel_x)
        figure;
        plot(timestamp, a_vel_x, 'LineWidth', 1.5);
        xlabel('Timestamp (s)');
        ylabel('a_vel_x');
        title('a_vel_x vs. Timestamp');
        grid on;
    else
        disp('Veriler boş olabilir, lütfen dosya formatını kontrol edin.');
    end
end






