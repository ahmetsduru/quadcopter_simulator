function actual_acc_y_t(log_folder)
    % Dosya yolunu belirleme (log klasöründen okuma)
    file_path = fullfile(log_folder, 'states.txt');

    % Veriyi okuma (readtable kullanarak daha esnek şekilde okuma)
    data = readtable(file_path, 'Delimiter', ',');

    % Timestamp (1. kolon) ve a_acc_y (9. kolon) verilerini ayırma
    timestamp = data{:, 1};  % 1. kolon: timestamp
    a_acc_y = data{:, 9};      % 9. kolon: a_acc_y

    % Eğer veriler doğruysa, grafiği çizelim
    if ~isempty(timestamp) && ~isempty(a_acc_y)
        figure;
        plot(timestamp, a_acc_y, 'LineWidth', 1.5);
        xlabel('Timestamp (s)');
        ylabel('a_acc_y');
        title('a_acc_y vs. Timestamp');
        grid on;
    else
        disp('Veriler boş olabilir, lütfen dosya formatını kontrol edin.');
    end
end