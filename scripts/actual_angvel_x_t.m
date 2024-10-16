function actual_angvel_x_t(log_folder)
    % Dosya yolunu belirleme (log klasöründen okuma)
    file_path = fullfile(log_folder, 'states.txt');

    % Veriyi okuma (readtable kullanarak daha esnek şekilde okuma)
    data = readtable(file_path, 'Delimiter', ',');

    % Timestamp (1. kolon) ve a_angvel_x (15. kolon) verilerini ayırma
    timestamp = data{:, 1};  % 1. kolon: timestamp
    a_angvel_x = data{:, 15};      % 15. kolon: a_angvel_x

    % Eğer veriler doğruysa, grafiği çizelim
    if ~isempty(timestamp) && ~isempty(a_angvel_x)
        figure;
        plot(timestamp, a_angvel_x, 'LineWidth', 1.5);
        xlabel('Timestamp (s)');
        ylabel('a_angvel_x');
        title('a_angvel_x vs. Timestamp');
        grid on;
    else
        disp('Veriler boş olabilir, lütfen dosya formatını kontrol edin.');
    end
end
